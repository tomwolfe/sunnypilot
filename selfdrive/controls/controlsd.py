#!/usr/bin/env python3
import math
import threading
import time
from numbers import Number
import numpy as np

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.monitoring.safety_monitor import SafetyMonitor
from openpilot.common.performance_monitor import PerfTrack, perf_monitor

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls(ControlsExt, ModelStateBase):
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.params)
    ModelStateBase.__init__(self)

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    # Safety Threshold Definitions for overall_safety_score:
    # 0.0-0.2: Immediate disengagement - System is in a critical state, immediate human intervention is required.
    # 0.2-0.4: Strong intervention (reduced speed, conservative control) - System detects high risk, takes strong
    #          measures to mitigate, but may not fully disengage.
    # 0.4-0.6: Degraded mode (reduced acceleration, enhanced caution) - System is operating with reduced confidence,
    #          applies more cautious driving behaviors.
    # 0.6+: Normal operation - System is functioning as expected with high confidence.
    #
    # Initialize configurable safety thresholds
    try:
        critical_threshold_param = self.params.get("SafetyCriticalThreshold")
        self.safety_critical_threshold = float(critical_threshold_param) if critical_threshold_param else 0.2
        # Add range validation: critical threshold should be between 0 and 1
        if self.safety_critical_threshold < 0.0 or self.safety_critical_threshold > 1.0:
            cloudlog.warning(f"Invalid SafetyCriticalThreshold: {self.safety_critical_threshold}, using default 0.2")
            self.safety_critical_threshold = 0.2
    except (TypeError, ValueError):
        self.safety_critical_threshold = 0.2  # Default value if parameter is invalid type

    try:
        high_risk_threshold_param = self.params.get("SafetyHighRiskThreshold")
        self.safety_high_risk_threshold = float(high_risk_threshold_param) if high_risk_threshold_param else 0.4
        # Add range validation: high risk threshold should be between 0 and 1 and strictly greater than critical
        if not (0.0 <= self.safety_high_risk_threshold <= 1.0):
            cloudlog.warning(f"SafetyHighRiskThreshold {self.safety_high_risk_threshold} is out of range [0, 1]. Using default 0.4")
            self.safety_high_risk_threshold = 0.4
        elif self.safety_high_risk_threshold <= (self.safety_critical_threshold + 0.1):
            cloudlog.warning(f"SafetyHighRiskThreshold {self.safety_high_risk_threshold} must be at least 0.1 greater than SafetyCriticalThreshold {self.safety_critical_threshold}. Using default 0.4")
            self.safety_high_risk_threshold = 0.4
    except (TypeError, ValueError):
        self.safety_high_risk_threshold = 0.4  # Default value if parameter is invalid type

    try:
        moderate_risk_threshold_param = self.params.get("SafetyModerateRiskThreshold")
        self.safety_moderate_risk_threshold = float(moderate_risk_threshold_param) if moderate_risk_threshold_param else 0.6
        # Add range validation: moderate threshold should be between 0 and 1 and strictly greater than high risk
        if not (0.0 <= self.safety_moderate_risk_threshold <= 1.0):
            cloudlog.warning(f"SafetyModerateRiskThreshold {self.safety_moderate_risk_threshold} is out of range [0, 1]. Using default 0.6")
            self.safety_moderate_risk_threshold = 0.6
        elif self.safety_moderate_risk_threshold <= self.safety_high_risk_threshold:
            cloudlog.warning(f"SafetyModerateRiskThreshold {self.safety_moderate_risk_threshold} is not strictly greater than SafetyHighRiskThreshold {self.safety_high_risk_threshold}. Using default 0.6")
            self.safety_moderate_risk_threshold = 0.6
    except (TypeError, ValueError):
        self.safety_moderate_risk_threshold = 0.6  # Default value if parameter is invalid type

    self.sm = messaging.SubMaster(['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveDelay',
                                   'radarState'] + self.sm_services_ext,
                                  poll=['selfdriveState', 'radarState', 'livePose', 'modelV2', 'carState', 'carControl'])
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
      self.LaC = LatControlAngle(self.CP, self.CP_SP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CP_SP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CP_SP, self.CI)

    # Pre-allocate reusable arrays to reduce allocation
    self._angle_array = np.zeros(3, dtype=np.float32)  # For orientation/angle data
    self._angular_velocity_array = np.zeros(3, dtype=np.float32)  # For angular velocity data

    # Initialize safety monitor for enhanced safety checks
    self.safety_monitor = SafetyMonitor()
    self.safety_degraded_mode = False

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']

    # Initialize safety report to default values
    safety_report = {}
    requires_intervention = False

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    # Use cached math operations - optimize radians calculation
    self.steer_angle_without_offset_cached = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(self.steer_angle_without_offset_cached, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

        self.LaC.extension.update_limits()

      self.LaC.extension.update_model_v2(self.sm['modelV2'])

      self.LaC.extension.update_lateral_lag(self.lat_delay)

    # Enhanced safety monitoring - perform safety checks using multi-sensor fusion
    if self.sm.all_checks(['modelV2', 'radarState', 'carState', 'carControl', 'livePose']):
      try:
        safety_score, requires_intervention, safety_report = self.safety_monitor.update(
          self.sm['modelV2'],
          self.sm['radarState'],
          self.sm['carState'],
          self.sm['carControl'],
          self.sm['livePose']
        )

        # Update safety degraded mode based on safety monitor
        self.safety_degraded_mode = safety_report.get('confidence_degraded', False)

        # Check for critical safety failures that require immediate disengagement
        overall_safety_score = safety_report.get('overall_safety_score', 1.0)
        if overall_safety_score < self.safety_critical_threshold:  # Critical safety threshold
          cloudlog.error(f"Critical safety failure: safety score {overall_safety_score} < {self.safety_critical_threshold} - requesting disengagement")
          # Request immediate disengagement for critical safety failures
          self.safety_degraded_mode = True
          requires_intervention = True
        elif overall_safety_score < self.safety_high_risk_threshold:  # High risk threshold
          cloudlog.warning(f"High risk safety level: safety score {overall_safety_score} < {self.safety_high_risk_threshold} - applying strong safety measures")
          self.safety_degraded_mode = True

        # If intervention is required, apply safety measures
        if requires_intervention:
          # Reduce acceleration limits for safer deceleration
          cloudlog.warning("Safety intervention required - applying conservative driving")
      except Exception as e:
        cloudlog.error(f"Safety monitor update failed: {e}")
        # Log detailed error information
        import traceback
        cloudlog.error(f"Safety monitor error traceback: {traceback.format_exc()}")
        # Default to safe state if monitoring fails
        self.safety_degraded_mode = True
        safety_report = {'overall_safety_score': 0.1, 'confidence_degraded': True}
        # Safety report should include failure information
        safety_report['error_occurred'] = True
        safety_report['error_details'] = str(e)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    self._standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill  # cache this calculation

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not self._standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
                    (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing - optimize by checking if necessary
    lane_change_state = model_v2.meta.laneChangeState
    if lane_change_state != LaneChangeState.off:
      lane_change_direction = model_v2.meta.laneChangeDirection  # cache this lookup
      CC.leftBlinker = lane_change_direction == LaneChangeDirection.left
      CC.rightBlinker = lane_change_direction == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)

    # Apply safety-based acceleration limits if in degraded mode
    if self.safety_degraded_mode:
      # Get overall safety score
      overall_safety_score = safety_report.get('overall_safety_score', 1.0)

      # Determine appropriate safety factor based on safety score
      if overall_safety_score < self.safety_critical_threshold:  # Critical safety threshold
        # Very conservative operation, approaching disengagement
        acceleration_safety_factor = 0.5
        braking_safety_factor = 0.7  # Less reduction on braking to maintain stopping capability
        # Consider disengaging if safety score remains critically low
        cloudlog.warning(f"Critical safety mode: safety score {overall_safety_score} - very conservative operation")
      elif overall_safety_score < self.safety_high_risk_threshold:  # High risk threshold
        acceleration_safety_factor = 0.6
        braking_safety_factor = 0.8  # Slightly less reduction on braking
      elif overall_safety_score < self.safety_moderate_risk_threshold:  # Moderate risk threshold
        acceleration_safety_factor = 0.75
        braking_safety_factor = 0.85  # Minimal reduction on braking
      else:  # Lower risk but still degraded
        acceleration_safety_factor = 0.85
        braking_safety_factor = 0.95  # Very minimal reduction on braking

      # Apply safety factors to acceleration limits while preserving braking capability
      # Note: pid_accel_limits[0] is typically the minimum (braking) value (negative)
      #       pid_accel_limits[1] is typically the maximum (acceleration) value (positive)
      min_accel_limit = pid_accel_limits[0]  # Braking capability (typically negative value)
      max_accel_limit = pid_accel_limits[1]  # Acceleration capability (typically positive value)

      # Preserve braking capability more than acceleration for safety
      # For braking (negative values), we want to make the limit less negative (closer to 0) by a smaller factor
      # For acceleration (positive values), we reduce more significantly
      adjusted_min_limit = min_accel_limit * braking_safety_factor if min_accel_limit < 0 else min_accel_limit * braking_safety_factor
      adjusted_max_limit = max_accel_limit * acceleration_safety_factor if max_accel_limit > 0 else max_accel_limit * acceleration_safety_factor

      pid_accel_limits = (adjusted_min_limit, adjusted_max_limit)

    actuators.accel = self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits)  # removed float() wrapper

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited)  # TODO what if not available
    actuators.torque = steer  # removed float() wrapper for performance
    actuators.steeringAngleDeg = steeringAngleDeg  # removed float() wrapper for performance

    # Optimize finite check - avoid dictionary creation unless needed
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number) or math.isfinite(attr):
        continue

      cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
      setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      # Use pre-allocated arrays to avoid repeated list creation
      np.copyto(self._angle_array[:3], self.calibrated_pose.orientation.xyz)
      CC.orientationNED = self._angle_array[:3].tolist()
      np.copyto(self._angular_velocity_array[:3], self.calibrated_pose.angular_velocity.xyz)
      CC.angularVelocity = self._angular_velocity_array[:3].tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = CS.vCruiseCluster * CV.KPH_TO_MS  # Removed float() wrapper
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

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = self.LoC.pid.p  # Removed float() wrapper
    cs.uiAccelCmd = self.LoC.pid.i  # Removed float() wrapper
    cs.ufAccelCmd = self.LoC.pid.f  # Removed float() wrapper
    cs.forceDecel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                     (self.sm['selfdriveState'].state == State.softDisabling)  # Removed bool() wrapper

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
      self.get_params_sp()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, self.sm["liveDelay"].lateralDelay)

      time.sleep(0.1)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e,))
    try:
      t.start()
      while True:
        with PerfTrack("controlsd_update"):
          self.update()
        with PerfTrack("controlsd_state_control") as perf_tracker:
          CC, lac_log = self.state_control()
        with PerfTrack("controlsd_publish"):
          self.publish(CC, lac_log)
        with PerfTrack("controlsd_ext"):
          self.run_ext(self.sm, self.pm)
        rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
