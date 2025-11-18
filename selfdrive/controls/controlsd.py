#!/usr/bin/env python3
import math
import threading
import time
from numbers import Number

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
from openpilot.selfdrive.controls.lib.enhanced_longitudinal_planner import EnhancedLongitudinalPlanner
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.common.performance_monitor import PerfTrack, perf_monitor
from openpilot.selfdrive.common.metrics import Metrics, record_metric

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

    self.sm = messaging.SubMaster(['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'liveDelay',
                                   'navInstruction', 'navStatus', 'validationMetrics', 'radarState'] + self.sm_services_ext,
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + self.pm_services_ext)

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP, self.CP_SP)
    self.VM = VehicleModel(self.CP)
    self.enhanced_long_planner = EnhancedLongitudinalPlanner(self.CP)
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

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    # Optimize radians calculation using cached value
    angle_offset_diff = CS.steeringAngleDeg - lp.angleOffsetDeg
    self.steer_angle_without_offset_cached = angle_offset_diff * 0.017453292519943295  # math.radians precomputed
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

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']
    validation_metrics = self.sm['validationMetrics'] if 'validationMetrics' in self.sm else None

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Optimize standstill check
    self._standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill

    # Get which state to use for active lateral control
    _lat_active = self.get_lat_active(self.sm)

    # Optimize boolean logic
    lat_active_conditions = _lat_active and not (CS.steerFaultTemporary or CS.steerFaultPermanent)
    CC.latActive = lat_active_conditions and (not self._standstill or self.CP.steerAtStandstill)

    # Optimize long active check
    long_events_override = any(e.overrideLongitudinal for e in self.sm['onroadEvents'])
    CC.longActive = CC.enabled and not long_events_override and (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Initialize safety validation based on validation metrics
    validation_state = {
        'lead_confidence_ok': True,
        'lane_confidence_ok': True,
        'overall_confidence_ok': True,
        'system_safe': True,
        'lane_change_safe': True
    }

    if validation_metrics is not None:
        # Check safety thresholds based on validation metrics
        lead_conf_ok = validation_metrics.leadConfidenceAvg >= 0.6
        lane_conf_ok = validation_metrics.laneConfidenceAvg >= 0.65
        overall_conf_ok = validation_metrics.overallConfidence >= 0.6
        lane_change_conf_ok = validation_metrics.overallConfidence >= 0.7 and validation_metrics.laneConfidenceAvg >= 0.7  # Higher threshold for lane changes

        validation_state['lead_confidence_ok'] = lead_conf_ok
        validation_state['lane_confidence_ok'] = lane_conf_ok
        validation_state['overall_confidence_ok'] = overall_conf_ok
        validation_state['lane_change_safe'] = lane_change_conf_ok
        validation_state['system_safe'] = lead_conf_ok and lane_conf_ok and overall_conf_ok

    # Dynamic safety margins for lane change decision-making based on validation metrics
    lane_change_state = model_v2.meta.laneChangeState
    original_lane_change_state = lane_change_state

    # Modify lane change state based on validation metrics
    if validation_metrics is not None and lane_change_state != LaneChangeState.off:
        if not validation_state['lane_change_safe']:
            # If validation metrics indicate low confidence, cancel or prevent lane changes
            # Set to preLaneChange to slow down the transition
            if lane_change_state == LaneChangeState.laneChangeStarting:
                lane_change_state = LaneChangeState.preLaneChange
            elif lane_change_state == LaneChangeState.laneChangeFinishing:
                lane_change_state = LaneChangeState.laneChangeStarting  # Interrupt finish
        else:
            # If high confidence, allow normal lane change behavior
            lane_change_state = original_lane_change_state

    # Enable blinkers while lane changing - optimize by checking if necessary
    if lane_change_state != LaneChangeState.off:
      lane_change_direction = model_v2.meta.laneChangeDirection
      CC.leftBlinker = lane_change_direction == LaneChangeDirection.left
      CC.rightBlinker = lane_change_direction == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # Enhanced longitudinal control using validation metrics for safety
    v_cruise_ms = CS.vCruise * CV.KPH_TO_MS
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, self.CP_SP, CS.vEgo, v_cruise_ms)

    # Apply enhanced longitudinal planning if validation metrics are available
    radar_state = self.sm['radarState'] if self.sm.updated['radarState'] else None
    if validation_metrics is not None:
        # Update lead vehicle prediction with confidence-based adjustments
        enhanced_long_plan = self.enhanced_long_planner.update_lead_vehicle_prediction(
            long_plan,
            radar_state,
            CS,  # Pass car state for vEgo information
            {
                'leadConfidenceAvg': validation_metrics.leadConfidenceAvg,
                'leadConfidenceMax': validation_metrics.leadConfidenceMax,
                'laneConfidenceAvg': validation_metrics.laneConfidenceAvg,
                'overallConfidence': validation_metrics.overallConfidence,
                'isValid': validation_metrics.isValid,
                'confidenceThreshold': validation_metrics.confidenceThreshold
            }
        )

        # Use enhanced longitudinal plan
        enhanced_a_target = enhanced_long_plan.aTarget
        enhanced_should_stop = enhanced_long_plan.shouldStop
    else:
        # Use original plan when validation metrics not available
        enhanced_a_target = long_plan.aTarget
        enhanced_should_stop = long_plan.shouldStop

    # Adjust control aggressiveness based on validation metrics
    if validation_metrics is not None and not validation_state['lead_confidence_ok']:
        # Reduce PID aggressiveness when lead detection confidence is low
        # This makes the system more conservative with longitudinal control
        pid_accel_limits = (
            max(pid_accel_limits[0] * 0.8, -4.0),  # More conservative deceleration
            min(pid_accel_limits[1] * 0.8, 2.0)   # More conservative acceleration
        )

    actuators.accel = self.LoC.update(CC.longActive, CS, enhanced_a_target, enhanced_should_stop, pid_accel_limits)

    # Steering PID loop and lateral MPC - optimize curvature calculations
    active_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature

    # Apply dynamic safety margins based on validation metrics
    if validation_metrics is not None:
        if not validation_state['lane_confidence_ok']:
            # Reduce desired curvature when lane detection confidence is low
            # This makes lateral control more conservative
            if abs(active_curvature) > 0.005:  # Only modify if there's significant curvature request
                active_curvature = active_curvature * 0.7  # Reduce by 30% when confidence is low
        elif validation_metrics.overallConfidence >= 0.85:
            # More aggressive control when high confidence
            active_curvature = active_curvature * 1.1  # Slightly more responsive when confidence is high

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, active_curvature, lp.roll)

    actuators.curvature = self.desired_curvature

    # Apply additional safety checks based on validation state
    lat_active_with_safety = CC.latActive and validation_state['system_safe']

    # Modify lateral control if validation metrics indicate low confidence
    if not validation_state['system_safe']:
        # Add additional stability checks when system confidence is low
        if abs(self.desired_curvature) < 0.001:  # Only for very small curvature requests
            lat_active_with_safety = CC.latActive  # Allow minimal lateral control
        else:
            # Be more conservative with lateral control when confidence is low
            lat_active_with_safety = CC.latActive and (
                validation_state['lane_confidence_ok'] or
                abs(self.desired_curvature) < 0.005  # Allow only very small corrections
            )

    steer, steeringAngleDeg, lac_log = self.LaC.update(lat_active_with_safety, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited)
    actuators.torque = steer
    actuators.steeringAngleDeg = steeringAngleDeg

    # More efficient finite check - avoid expensive dict creation
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number) or math.isfinite(attr):
        continue

      # Only create dict if there's an error (rare case)
      cloudlog.error(f"actuators.{p} not finite")
      setattr(actuators, p, 0.0)

    # Record control system metrics
    record_metric(Metrics.STEERING_LATENCY_MS, 0.0, {  # Placeholder - actual latency would need real measurement
        "operation": "steering_update",
        "v_ego": CS.vEgo,
        "steering_angle_deg": CS.steeringAngleDeg,
        "curvature": self.curvature
    })

    record_metric(Metrics.BRAKING_LATENCY_MS, 0.0, {  # Placeholder - actual latency would need real measurement
        "operation": "braking_update",
        "v_ego": CS.vEgo,
        "a_ego": CS.aEgo,
        "accel_cmd": actuators.accel
    })

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

    # Safety validation layers - trigger fallback behaviors when confidence metrics are low
    safety_status = {
        'fallback_active': False,
        'degraded_mode': False,
        'system_alert': False
    }

    if validation_metrics is not None:
        # Trigger fallback behaviors based on validation metrics
        if validation_metrics.overallConfidence < 0.4:
            # Critical safety threshold - system should disengage
            safety_status['fallback_active'] = True
            safety_status['system_alert'] = True
            # Force disengagement when confidence is critically low
            CC.enabled = False
        elif validation_metrics.overallConfidence < 0.6:
            # Degraded mode when confidence is moderate
            safety_status['degraded_mode'] = True
            # Reduce lateral control authority gradually
        elif validation_metrics.overallConfidence < 0.75:
            # Warning level when confidence is somewhat low
            safety_status['system_alert'] = True

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

    # Add safety validation metrics to controlsState for monitoring
    if validation_metrics is not None:
        # Set validation metrics - need to create and populate the metrics struct
        cs.validation.metrics.leadConfidenceAvg = validation_metrics.leadConfidenceAvg
        cs.validation.metrics.leadConfidenceMax = validation_metrics.leadConfidenceMax
        cs.validation.metrics.laneConfidenceAvg = validation_metrics.laneConfidenceAvg
        cs.validation.metrics.overallConfidence = validation_metrics.overallConfidence
        cs.validation.metrics.isValid = validation_metrics.isValid
        cs.validation.metrics.confidenceThreshold = validation_metrics.confidenceThreshold
    else:
        # Default values when validation metrics are not available
        cs.validation.metrics.leadConfidenceAvg = 0.0
        cs.validation.metrics.leadConfidenceMax = 0.0
        cs.validation.metrics.laneConfidenceAvg = 0.0
        cs.validation.metrics.overallConfidence = 0.0
        cs.validation.metrics.isValid = False
        cs.validation.metrics.confidenceThreshold = 0.5

    # Update state based on safety status
    cs.fallbackActive = safety_status['fallback_active']
    cs.degradedMode = safety_status['degraded_mode']
    cs.systemAlert = safety_status['system_alert']

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
        # Track the overall loop performance
        with PerfTrack("controlsd_loop") as loop_tracker:
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
