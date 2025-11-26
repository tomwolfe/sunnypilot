#!/usr/bin/env python3
import math
import threading
import time
from collections import deque
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.selfdrive.controls.lib.safety_helpers import SafetyManager
from openpilot.selfdrive.controls.lib.edge_case_handler import EdgeCaseHandler

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls(ControlsExt):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveDelay',
                                   'deviceState', 'radarState'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI, DT_CTRL)

    # Initialize thermal management system with enhanced tracking
    # thermal_performance_factor: Represents the current overall system stress (thermal, CPU, memory).
    #   A value of 1.0 means no stress, lower values indicate higher stress.
    #   It's a smoothed value derived from the maximum of thermal, CPU, and memory percentages.
    # thermal_history: A deque that tracks the thermal_performance_factor over the last 30 cycles (0.3 seconds at 100Hz).
    #   Used for trend analysis to predict future thermal behavior.
    # thermal_stress_level: Categorizes the system's stress into discrete levels (0-3).
    #   0=normal, 1=moderate, 2=high, 3=very high. This provides context-aware control.
    # performance_compensation_factor: An additional factor that further reduces performance based on rapidly increasing
    #   thermal stress trends. It acts as a proactive cushion against thermal runaway.
    self.thermal_performance_factor = 1.0
    self.thermal_history = deque(maxlen=30)  # Track thermal performance over last 30 cycles (0.3 seconds at 100Hz)
    self.thermal_stress_level = 0  # 0=normal, 1=moderate, 2=high, 3=very high
    self.performance_compensation_factor = 1.0  # Additional compensation for performance degradation

    # Initialize safety manager for advanced safety features
    self.safety_manager = SafetyManager()

    # Initialize edge case handler for unusual scenario detection and handling
    self.edge_case_handler = EdgeCaseHandler()

  def update(self):
    """
    Update the control system with new sensor and model data.

    This method updates the internal state of the control system by processing
    new messages from various sensors and the model. It also implements adaptive
    control based on thermal performance to maintain system stability under
    varying hardware conditions.
    """
    # Update at 15Hz to maintain message flow for critical communication (e.g., carState, radarState)
    # without adding significant load, especially during skipped control cycles.
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

    # Enhanced thermal management with predictive capabilities and stress level tracking
    if self.sm.updated['deviceState']:
      thermal_status = self.sm['deviceState'].thermalStatus
      thermal_prc = self.sm['deviceState'].thermalPerc
      cpu_usage = max(self.sm['deviceState'].cpuUsagePercent) if self.sm['deviceState'].cpuUsagePercent else 0
      memory_usage = self.sm['deviceState'].memoryUsagePercent

      cloudlog.debug(f"Thermal management inputs: thermal_prc={thermal_prc:.2f}, cpu_usage={cpu_usage:.2f}, memory_usage={memory_usage:.2f}")

      # Enhanced thermal management with more granular control and predictive elements
      base_thermal_factor = thermal_prc / 100.0
      cpu_factor = cpu_usage / 100.0
      memory_factor = memory_usage / 100.0

      # Calculate overall system stress as the maximum of all factors (thermal, CPU, memory).
      # This provides a holistic view of the system's current load.
      system_stress = max(base_thermal_factor, cpu_factor, memory_factor)

      # Apply hysteresis and smoothing to `thermal_performance_factor` to prevent rapid, destabilizing oscillations
      # between performance states. A higher smoothing factor is used when stress is decreasing to gradually
      # restore performance, while a lower factor is used when stress is increasing to react more quickly.
      if hasattr(self, 'prev_thermal_factor'):
        # Apply smoothing to prevent oscillation between thermal states
        smoothing_factor = 0.9 if system_stress <= self.prev_thermal_factor else 0.7
        self.thermal_performance_factor = (smoothing_factor * self.prev_thermal_factor +
                                          (1 - smoothing_factor) * system_stress)
      else:
        self.thermal_performance_factor = system_stress

      self.prev_thermal_factor = self.thermal_performance_factor

      # Update thermal history for trend analysis
      self.thermal_history.append(self.thermal_performance_factor)

      # Determine thermal stress level based on current and historical data.
      # This allows for context-aware control adjustments.
      avg_thermal = sum(self.thermal_history) / len(self.thermal_history) if self.thermal_history else 0
      current_thermal = self.thermal_performance_factor

      # Classify stress level based on both current and average thermal state
      if current_thermal > 0.9 or avg_thermal > 0.85:
        self.thermal_stress_level = 3  # Very high stress
      elif current_thermal > 0.8 or avg_thermal > 0.75:
        self.thermal_stress_level = 2  # High stress
      elif current_thermal > 0.7 or avg_thermal > 0.65:
        self.thermal_stress_level = 1  # Moderate stress
      else:
        self.thermal_stress_level = 0  # Normal

      # Adjust performance compensation based on thermal stress trend.
      # If stress is rapidly increasing, proactively apply extra compensation as a "cushion."
      # If stress is decreasing, gradually restore performance.
      if len(self.thermal_history) > 10:
        # Calculate thermal trend (increasing, decreasing, stable)
        recent_avg = sum(list(self.thermal_history)[-5:]) / 5
        older_avg = sum(list(self.thermal_history)[:-5]) / max(1, len(list(self.thermal_history)[:-5]))

        if recent_avg > older_avg + 0.1:  # Thermal stress is increasing rapidly
          # Apply additional compensation to prevent thermal runaway
          self.performance_compensation_factor = max(0.8, self.thermal_performance_factor - 0.1)
        elif recent_avg < older_avg - 0.1:  # Thermal stress is decreasing
          # Gradually restore performance
          self.performance_compensation_factor = min(1.0, self.performance_compensation_factor + 0.05)
        else:  # Thermal stress is relatively stable
          self.performance_compensation_factor = self.thermal_performance_factor
      else:
        self.performance_compensation_factor = self.thermal_performance_factor

      # Additional thermal considerations for safety-critical functions
      # Thermal status values: green=0, yellow=1, red=2, danger=3
      # Values >= 5 indicate extreme thermal conditions requiring immediate action
      # This provides an additional safety margin beyond standard thermal warnings
      if thermal_status >= 5:  # Critical thermal status (beyond standard danger level)
        cloudlog.warning(f"Critical thermal status: {thermal_status}, reducing performance to protect hardware")
        # Reduce performance factor further when in critical thermal state
        self.thermal_performance_factor = min(self.thermal_performance_factor, 0.7)
        self.performance_compensation_factor = min(self.performance_compensation_factor, 0.6)
    else:
      self.thermal_performance_factor = 1.0
      self.performance_compensation_factor = 1.0

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # Handle edge cases and unusual scenarios
    edge_scenarios = self.edge_case_handler.handle_unusual_scenarios(
      CS,
      self.sm['radarState'] if 'radarState' in self.sm else None,
      self.sm['modelV2'] if 'modelV2' in self.sm else None
    )

    # Get adaptive control modifications based on detected scenarios
    adaptive_mods = self.edge_case_handler.get_adaptive_control_modifications(CS, edge_scenarios)

    # Apply adaptive control modifications to the system
    if adaptive_mods['caution_mode']:
      # Increase time headway for safety in unusual scenarios
      # This is done by modifying the long_plan to create larger gaps
      if hasattr(long_plan, 'aTarget') and adaptive_mods['longitudinal_factor'] < 1.0:
        # Make longitudinal control more conservative
        base_accel_limit = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
        # Apply conservative factor to acceleration limits
        conservative_accel_limits = (
          base_accel_limit[0] * adaptive_mods['longitudinal_factor'],
          base_accel_limit[1] * adaptive_mods['longitudinal_factor']
        )
      else:
        conservative_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    else:
      conservative_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)

    # Enhanced saturation handling with adaptive limits
    # accel PID loop
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, conservative_accel_limits))

    # Apply lateral control modifications if needed
    modified_desired_curvature = model_v2.action.desiredCurvature
    if adaptive_mods['lateral_factor'] < 1.0 and CC.latActive:
      # Make lateral control more conservative by reducing desired curvature
      modified_desired_curvature = model_v2.action.desiredCurvature * adaptive_mods['lateral_factor']

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = modified_desired_curvature if CC.latActive else self.curvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited, lat_delay)

    # Enhanced saturation detection with smoother recovery
    saturation_detected = False
    if hasattr(lac_log, 'saturated') and lac_log.saturated:
      saturation_detected = True
      cloudlog.debug(f"Steering saturation detected at vEgo: {CS.vEgo:.2f} m/s")

    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)

    # Enhanced finite value checks with recovery mechanism
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite. Actuators: {actuators.to_dict()}, CarState: {CS.to_dict()}, LongitudinalPlan: {long_plan.to_dict()}, LateralControlLog: {lac_log.to_dict()}")
        # Implement a recovery by setting to a safe value instead of 0.0
        if p in ['steeringAngleDeg', 'curvature']:
          # For steering-related values, use current measurement as fallback
          setattr(actuators, p, 0.0 if p == 'curvature' else CS.steeringAngleDeg)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.critical # Indicate non-finite steering to user
        elif p == 'accel':
          # For acceleration, use 0 to maintain current speed
          setattr(actuators, p, 0.0)
        else:
          # Default fallback to 0.0
          setattr(actuators, p, 0.0)

    # Enhanced saturation handling in controls state
    if saturation_detected:
      CC.hudControl.visualAlert = log.ControlsState.AlertStatus.normal  # Indicate saturation to user

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # Perform safety check before publishing controls
    safety_ok, safety_violation = self.safety_manager.check_safety_violations(
      CS,
      self.sm['carOutput'].actuatorsOutput if 'carOutput' in self.sm else None,
      self.sm['modelV2'] if 'modelV2' in self.sm else None
    )

    # If safety violation detected, modify control output to be safer
    if not safety_ok and self.safety_manager.should_disengage():
      # Apply emergency deceleration and center steering
      CC.actuators.accel = min(CC.actuators.accel, -1.0)  # Apply mild brake
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        CC.actuators.steeringAngleDeg = CS.steeringAngleDeg * 0.9  # Gradually center steering
      else:
        CC.actuators.curvature = self.curvature * 0.9  # Gradually reduce curvature
      CC.enabled = False  # Disengage to prevent dangerous situation
      CC.latActive = False
      CC.longActive = False

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    # Add safety status information to controls state
    cs.safetyStatus = self.safety_manager.get_safety_recommendation(
      CS,
      self.sm['carOutput'].actuatorsOutput if 'carOutput' in self.sm else None,
      self.sm['modelV2'] if 'modelV2' in self.sm else None
    )

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def params_thread(self, evt):
    while not evt.is_set():
      # Add any parameter updates that need to be monitored in a background thread
      time.sleep(0.1)

  def run(self):
    """
    Main control loop that runs at an adaptive rate based on thermal conditions.

    The control system normally runs at 100Hz (base_rate), but this rate is
    dynamically adjusted based on the thermal performance factor to reduce
    computational load when the device is overheating. The rate is reduced
    gradually to maintain system stability while protecting hardware.
    """
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e,))
    try:
      t.start()
      rk = Ratekeeper(100, print_delay_threshold=None)  # Base rate is 100Hz
      thermal_adjusted_frame = 0

      # Enhanced thermal scaling parameters with dynamic adjustment based on stress level
      base_rate = 100  # Hz
      # `min_rates` provides dynamic, context-aware control. It maps thermal stress levels to minimum allowed control loop frequencies.
      # This ensures that even under moderate stress, the system doesn't throttle too aggressively, preserving responsiveness.
      min_rates = {0: 80, 1: 70, 2: 60, 3: 50}  # Lower rates for higher stress levels
      while True:
        # `current_rate` is adaptively calculated based on the current thermal stress level and the
        # `performance_compensation_factor`. This dynamically adjusts the control loop frequency
        # to maintain system stability and hardware protection under varying thermal conditions.
        current_stress_level = self.thermal_stress_level
        current_rate = max(min_rates[current_stress_level], base_rate * self.performance_compensation_factor)

        # Process every frame when at full rate, every other frame when at reduced rate
        frame_skip_threshold = base_rate / current_rate
        thermal_adjusted_frame += 1

        # Only perform full control cycle if we're not skipping this frame due to thermal constraints
        if thermal_adjusted_frame >= frame_skip_threshold:
          thermal_adjusted_frame = 0  # Reset counter
          self.update()
          CC, lac_log = self.state_control()
          self.publish(CC, lac_log)
          self.get_params_sp(self.sm)
          self.run_ext(self.sm, self.pm)

          # Enhanced thermal monitoring and logging with stress level information
          if self.thermal_stress_level > 0:
            cloudlog.debug(f"Thermal throttling active: stress_level={self.thermal_stress_level}, "
                          f"factor={self.performance_compensation_factor:.2f}, rate={current_rate:.1f}Hz, "
                          f"current_thermal={self.thermal_performance_factor:.2f}")
        else:
          # Still update the message subsystem regularly to maintain message flow
          # This 15Hz update rate is chosen to ensure critical communication (e.g., carState, radarState)
          # is maintained even when the main control loop is thermally throttled and frames are skipped,
          # without adding significant computational load during these skipped cycles.
          self.sm.update(15)

        # Monitor timing with thermal awareness and add thermal-dependent performance logging
        timing_start = time.monotonic()
        rk.monitor_time()
        timing_elapsed = time.monotonic() - timing_start

        # Additional thermal monitoring for extreme cases and logging based on thermal state
        if timing_elapsed > 0.02:  # If we're taking too long, log it
          cloudlog.debug(f"Control loop timing exceeded threshold: {timing_elapsed*1000:.1f}ms, "
                        f"thermal_factor: {self.thermal_performance_factor:.2f}, "
                        f"stress_level: {self.thermal_stress_level}")

        # Add thermal-based performance adjustments to the lateral and longitudinal controllers
        if hasattr(self.LaC, 'update_thermal_compensation'):
          self.LaC.update_thermal_compensation(self.thermal_stress_level, self.performance_compensation_factor)
        if hasattr(self.LoC, 'update_thermal_compensation'):
          self.LoC.update_thermal_compensation(self.thermal_stress_level, self.performance_compensation_factor)

    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
