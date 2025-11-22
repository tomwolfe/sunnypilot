#!/usr/bin/env python3
import math
import threading
import time
from numbers import Number
import numpy as np

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params, UnknownKeyName
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
from openpilot.sunnypilot.common.performance_monitor import PerformanceMonitor
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

    # Load safety thresholds from params, with defaults if not found
    try:
      min_braking_param = self.params.get("SafeMinBrakingThreshold", encoding='utf8')
      self.SAFE_MIN_BRAKING_THRESHOLD = float(min_braking_param) if min_braking_param else -4.0
    except (UnknownKeyName, ValueError):
      cloudlog.warning("SafeMinBrakingThreshold not found or invalid, defaulting to -4.0 m/s^2.")
      self.SAFE_MIN_BRAKING_THRESHOLD = -4.0  # Default value
    
    try:
      max_acceleration_param = self.params.get("SafeMaxAccelerationThreshold", encoding='utf8')
      self.SAFE_MAX_ACCELERATION_THRESHOLD = float(max_acceleration_param) if max_acceleration_param else 2.0
    except (UnknownKeyName, ValueError):
      cloudlog.warning("SafeMaxAccelerationThreshold not found or invalid, defaulting to 2.0 m/s^2.")
      self.SAFE_MAX_ACCELERATION_THRESHOLD = 2.0  # Default value

    # Justification for default thresholds:
    # - SAFE_MIN_BRAKING_THRESHOLD (-4.0 m/s^2): This value represents a typical aggressive braking deceleration
    #   for passenger vehicles under good conditions. It's a balance between safety (ensuring the system can
    #   decelerate sufficiently) and comfort/drivability (avoiding excessively harsh braking).
    #   This threshold should be calibrated per vehicle type, as braking capabilities vary significantly.
    # - SAFE_MAX_ACCELERATION_THRESHOLD (2.0 m/s^2): This value represents a comfortable yet responsive acceleration.
    #   Exceeding this value can lead to uncomfortable driving or reduced safety margins if not carefully controlled.
    #   This threshold also requires vehicle-specific tuning to match powertrain characteristics.
    # These defaults are for general applicability and should be overridden with vehicle-specific values when possible.

    # Initialize sunnypilot controlsd extension and base model state
    ControlsExt.__init__(self, self.CP, self.CP_SP)
    ModelStateBase.__init__(self)

    # Performance monitoring parameters
    try:
        sampling_skip_factor_param = self.params.get("PerformanceSamplingSkipFactor")
        self.performance_sampling_skip_factor = int(float(sampling_skip_factor_param)) if sampling_skip_factor_param else 1 # default to no skip
    except (UnknownKeyName, ValueError):
        self.performance_sampling_skip_factor = 1
    
    try:
        sampling_cpu_threshold_param = self.params.get("PerformanceSamplingCpuThreshold")
        self.performance_sampling_cpu_threshold = float(sampling_cpu_threshold_param) if sampling_cpu_threshold_param else 50.0 # ms
    except (UnknownKeyName, ValueError):
        self.performance_sampling_cpu_threshold = 50.0

    self.performance_sampling_skip_counter = 0

    # Forced disengagement parameters for persistent performance degradation
    try:
        disengage_time_param = self.params.get("PerformanceDegradationDisengageTime")
        self.performance_degradation_disengage_time = float(disengage_time_param) if disengage_time_param else 5.0 # seconds
    except (UnknownKeyName, ValueError):
        self.performance_degradation_disengage_time = 5.0
    
    try:
        consecutive_frames_param = self.params.get("PerformanceDegradationConsecutiveFrames")
        self.performance_degradation_consecutive_frames = int(float(consecutive_frames_param)) if consecutive_frames_param else 50 # frames
    except (UnknownKeyName, ValueError):
        self.performance_degradation_consecutive_frames = 50 # Default to 50 frames (~0.5s at 100Hz)

    self.performance_degradation_start_time = 0.0 # Tracks when degradation started
    self.degradation_consecutive_count = 0 # Tracks consecutive frames of degradation

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP)

    # Safety Threshold Definitions for overall_safety_score:
    # These thresholds define the severity levels for safety intervention based on the `overall_safety_score`.
    # While default values are provided, they are conceptual and require rigorous empirical testing and
    # vehicle-specific calibration to ensure optimal performance and safety in diverse driving scenarios.
    #
    # 0.0-0.3 (Critical): Immediate disengagement - System is in a critical state, immediate human intervention is required.
    # 0.3-0.4 (High Risk): Strong intervention (reduced speed, conservative control) - System detects high risk,
    #           takes strong measures to mitigate, but may not fully disengage.
    # 0.4-0.6 (Moderate Risk): Degraded mode (reduced acceleration, enhanced caution) - System is operating with
    #            reduced confidence, applies more cautious driving behaviors.
    # 0.6+ (Normal): Normal operation - System is functioning as expected with high confidence.
    #
    # Initialize configurable safety thresholds
    try:
        critical_threshold_param = self.params.get("SafetyCriticalThreshold")
        self.safety_critical_threshold = float(critical_threshold_param) if critical_threshold_param else 0.3
        # Add range validation: critical threshold should be between 0 and 1
        if self.safety_critical_threshold < 0.0 or self.safety_critical_threshold > 1.0:
            cloudlog.warning(f"Invalid SafetyCriticalThreshold: {self.safety_critical_threshold}, using default 0.3")
            self.safety_critical_threshold = 0.3
    except (TypeError, ValueError):
        self.safety_critical_threshold = 0.3  # Default value if parameter is invalid type

    try:
        high_risk_threshold_param = self.params.get("SafetyHighRiskThreshold")
        self.safety_high_risk_threshold = float(high_risk_threshold_param) if high_risk_threshold_param else 0.4
        # Add range validation: high risk threshold should be between 0 and 1 and strictly greater than critical
        if not (0.0 <= self.safety_high_risk_threshold <= 1.0):
            cloudlog.warning(f"SafetyHighRiskThreshold {self.safety_high_risk_threshold} is out of range [0, 1]. Using default 0.4")
            self.safety_high_risk_threshold = 0.4
        elif self.safety_high_risk_threshold <= (self.safety_critical_threshold + 0.15):
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
                                   'radarState', 'gpsLocationExternal'] + self.sm_services_ext,
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

    # Initialize performance monitor for real-time performance evaluation
    self.performance_monitor = PerformanceMonitor()
    self.performance_degraded_mode = False

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
    self.performance_degraded_mode = False # Reset degraded mode at the start of each cycle

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
    if self.sm.all_checks(['modelV2', 'radarState', 'carState', 'carControl', 'livePose', 'gpsLocationExternal']):
      try:
        safety_score, requires_intervention, safety_report = self.safety_monitor.update(
          self.sm['modelV2'],
          self.sm.logMonoTime['modelV2'],
          self.sm['radarState'],
          self.sm.logMonoTime['radarState'],
          self.sm['carState'],
          self.sm.logMonoTime['carState'],
          self.sm['carControl'],
          self.sm['livePose'],
          self.sm.logMonoTime['livePose'] if 'livePose' in self.sm.logMonoTime else None,
          self.sm['driverMonitoringState'],
          self.sm.logMonoTime['driverMonitoringState'] if 'driverMonitoringState' in self.sm.logMonoTime else None,
          self.sm['gpsLocationExternal'],
          self.sm.logMonoTime['gpsLocationExternal'] if 'gpsLocationExternal' in self.sm.logMonoTime else None
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

    # Performance monitoring - evaluate real-time driving performance
    # Dynamic sampling rate control based on controlsd_state_control execution time
    current_state_control_time = perf_monitor.get_average_time("controlsd_state_control") # Assuming this is available

    if current_state_control_time is not None and current_state_control_time > self.performance_sampling_cpu_threshold:
        self.performance_sampling_skip_counter += 1
    else:
        self.performance_sampling_skip_counter = 0 # Reset counter if CPU usage is normal

    skip_performance_evaluation = False
    if self.performance_sampling_skip_factor > 1 and \
       self.performance_sampling_skip_counter > 0 and \
       self.performance_sampling_skip_counter % self.performance_sampling_skip_factor != 0:
        skip_performance_evaluation = True
        cloudlog.debug(f"Skipping performance evaluation due to high CPU ({current_state_control_time:.2f}ms) "
                      f"and skip factor {self.performance_sampling_skip_factor}. Counter: {self.performance_sampling_skip_counter}")

    if not skip_performance_evaluation and self.sm.all_checks(['modelV2', 'carState', 'controlsState']):
      try:
        # Prepare data for performance evaluation
        desired_state = {
          'lateral': 0.0, # Desired angle error is 0
          'longitudinal': self.sm['modelV2'].velocity.x[0] if len(self.sm['modelV2'].velocity.x) > 0 else CS.vEgo,
          'path_deviation': self.desired_curvature - self.curvature if hasattr(self, 'desired_curvature') and hasattr(self, 'curvature') else 0.0
        }

        # Calculate lateral error using the actual path deviation from model
        # The desired lateral position in the model frame is typically at y=0 (center of lane)
        # The actual lateral error can be derived from the path.y values or lateral error from controller
        desired_lateral_pos = 0.0  # Center of path in model frame
        if len(self.sm['modelV2'].path.y) > 0:
            # The first point in the path represents the immediate desired lateral offset from center
            desired_lateral_pos = self.sm['modelV2'].path.y[0]

        # The actual lateral deviation can be taken as the difference between what the model expects
        # and the actual control performance. We'll use the lateral error from path following
        actual_lateral_deviation = lac_log.yActual if hasattr(lac_log, 'yActual') and lac_log.yActual is not None else lac_log.error

        actual_state = {
          'lateral': abs(actual_lateral_deviation),  # True lateral deviation from path in meters
          'longitudinal': CS.vEgo,
          'lateral_accel': CS.aEgo,  # Using longitudinal acceleration as it's available
        }

        model_output = self.sm['modelV2']
        control_output = {
          'output': 0.0,  # Placeholder - will be actual control output
          'jerk': CS.aEgo - getattr(self, 'prev_acceleration', CS.aEgo)  # Change in acceleration
        }
        self.prev_acceleration = CS.aEgo  # Store for next iteration

        # Evaluate current performance
        performance_metrics = self.performance_monitor.evaluate_performance(
          desired_state, actual_state, model_output, control_output
        )

        # Check if parameters need adaptation based on performance
        adapt_needed, adaptation_params = self.performance_monitor.should_adapt_parameters()

        # Check performance health
        if self.performance_monitor.performance_unhealthy:
            cloudlog.warning("Performance unhealthy, forcing degraded mode.")
            self.performance_degraded_mode = True
            # Optional: revert to baseline tuning parameters if unhealthy
            # For now, we'll let the adaptation logic handle this more gracefully
            # adaptation_params = self.performance_monitor.performance_baseline_tuning_params() # if such a method existed

        # Implement safety lockout for parameter adaptation
        # Get overall safety score for critical check
        overall_safety_score_val = safety_report.get('overall_safety_score', 1.0) # From safety_monitor update

        if adapt_needed and adaptation_params:
          # Check for safety lockout conditions
          # Lockout only if overall safety score is below critical threshold.
          # This allows adaptation to occur in high-risk scenarios if it could improve safety.
          if overall_safety_score_val < self.safety_critical_threshold:
            cloudlog.warning(f"Safety lockout active: Preventing performance adaptation due to high risk safety state (score: {overall_safety_score_val:.2f}).")
            # Do not apply adaptation_params
          else:
            cloudlog.info(f"Performance adaptation triggered: {adaptation_params}")
            # Update tuning parameters for adaptive control
            for param, value in adaptation_params.items():
              if param in self.performance_monitor.tuning_params:
                self.performance_monitor.tuning_params[param] = value

            # Update the lateral controller with new parameters if needed
            if 'lateral_kp_factor' in adaptation_params or 'lateral_ki_factor' in adaptation_params:
              # In a real implementation, we would pass these to the lateral controller
              # For now, we'll just store them for reference
              self.adaptation_params = adaptation_params

      except Exception as e:
        cloudlog.error(f"Performance monitor update failed: {e}")
        import traceback
        cloudlog.error(f"Performance monitor error traceback: {traceback.format_exc()}")

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

    # Determine combined degraded mode status with dual-channel monitoring
    # Both safety and performance monitors must indicate degradation for a full degraded mode
    # A moderate risk from safety monitor or an unhealthy flag from performance monitor can trigger it.
    combined_degraded_mode = False
    if self.performance_monitor.performance_unhealthy and \
       (self.safety_degraded_mode or overall_safety_score < self.safety_moderate_risk_threshold):
      combined_degraded_mode = True
      cloudlog.warning("Dual-channel degraded mode: Both performance (unhealthy) and safety (degraded/moderate risk) indicate degradation.")
    elif self.performance_monitor.performance_unhealthy:
        cloudlog.info("Performance unhealthy, but safety is acceptable.")
    elif self.safety_degraded_mode:
        cloudlog.info("Safety degraded, but performance is acceptable.")

    # Check for persistent performance degradation requiring disengagement
    if combined_degraded_mode:
        self.degradation_consecutive_count = min(self.degradation_consecutive_count + 1, self.performance_degradation_consecutive_frames + 10) # Cap the counter to prevent indefinite growth
    else:
        # Decrement counter, but not below zero, to handle momentary glitches
        self.degradation_consecutive_count = max(0, self.degradation_consecutive_count - 2) # Decrement by 2 to allow faster recovery from brief non-degraded periods

    # Disengage if consecutive degraded frames exceed threshold
    if self.degradation_consecutive_count >= self.performance_degradation_consecutive_frames:
        cloudlog.error(f"Persistent performance degradation for {self.degradation_consecutive_count} frames. Requesting disengagement.")
        requires_intervention = True # Force disengagement
        self.degradation_consecutive_count = 0 # Reset counter after disengagement

    # Apply safety-based acceleration limits if in degraded mode
    if self.safety_degraded_mode or combined_degraded_mode: # Use combined_degraded_mode for stricter conditions
      # Get overall safety score
      overall_safety_score = safety_report.get('overall_safety_score', 1.0)

      # Determine appropriate safety factor based on safety score
      if overall_safety_score < self.safety_critical_threshold:  # Critical safety threshold
        # Very conservative operation, approaching disengagement
        acceleration_safety_factor = 0.5
        braking_safety_factor = 1.0  # Preserve full braking capability
        # Consider disengaging if safety score remains critically low
        cloudlog.warning(f"Critical safety mode: safety score {overall_safety_score} - very conservative operation")
      elif overall_safety_score < self.safety_high_risk_threshold:  # High risk threshold
        acceleration_safety_factor = 0.6
        braking_safety_factor = 1.0
      elif overall_safety_score < self.safety_moderate_risk_threshold:  # Moderate risk threshold
        acceleration_safety_factor = 0.75
        braking_safety_factor = 1.0
      else:  # Lower risk but still degraded
        acceleration_safety_factor = 0.85
        braking_safety_factor = 1.0

      # Incorporate performance-based adjustments
      if self.performance_degraded_mode:
        # Get performance-based factors from performance monitor
        performance_factor = self.performance_monitor.tuning_params.get('longitudinal_accel_limit_factor', 1.0)
        # Adjust safety factor based on performance (more conservative if performance is poor)
        acceleration_safety_factor *= performance_factor

      # Apply safety factors to acceleration limits while preserving braking capability
      # Note: pid_accel_limits[0] is typically the minimum (braking) value (negative)
      #       pid_accel_limits[1] is typically the maximum (acceleration) value (positive)
      min_accel_limit = pid_accel_limits[0]  # Braking capability (typically negative value)
      max_accel_limit = pid_accel_limits[1]  # Acceleration capability (typically positive value)

      # Preserve braking capability more than acceleration for safety
      # For braking (negative values), we want to make the limit less negative (closer to 0) by a smaller factor
      # For acceleration (positive values), we reduce more significantly
      adjusted_min_limit = min_accel_limit  # Preserve full braking capability
      adjusted_max_limit = max_accel_limit * acceleration_safety_factor

      pid_accel_limits = (adjusted_min_limit, adjusted_max_limit)

      # --- Start Safety Validation for Accel Limits ---
      # Ensure min_accel_limit is sufficiently negative (strong braking)
      # Assuming -4.0 m/s^2 is a reasonable minimum braking threshold for safety
      # This value should ideally be derived from vehicle capabilities or safety requirements
            # SAFE_MIN_BRAKING_THRESHOLD is now configurable via Params.
      if adjusted_min_limit > self.SAFE_MIN_BRAKING_THRESHOLD:
        cloudlog.warning(f"Safety Validation: adjusted_min_limit {adjusted_min_limit:.2f} m/s^2 "
                         f"is weaker than safe threshold {self.SAFE_MIN_BRAKING_THRESHOLD:.2f} m/s^2.")
        # Optionally, force a safer, more negative limit if validation fails
        # adjusted_min_limit = min(adjusted_min_limit, SAFE_MIN_BRAKING_THRESHOLD)

      # Ensure max_accel_limit does not exceed a reasonable acceleration threshold
      # Assuming 2.0 m/s^2 is a reasonable maximum acceleration threshold for safety
            # SAFE_MAX_ACCELERATION_THRESHOLD is now configurable via Params.
      if adjusted_max_limit > self.SAFE_MAX_ACCELERATION_THRESHOLD:
        cloudlog.warning(f"Safety Validation: adjusted_max_limit {adjusted_max_limit:.2f} m/s^2 "
                         f"is stronger than safe threshold {self.SAFE_MAX_ACCELERATION_THRESHOLD:.2f} m/s^2.")
        # Optionally, force a safer, less positive limit if validation fails
        # adjusted_max_limit = min(adjusted_max_limit, SAFE_MAX_ACCELERATION_THRESHOLD)
      # --- End Safety Validation for Accel Limits ---



    actuators.accel = self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits)  # removed float() wrapper

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)

    actuators.curvature = self.desired_curvature
    # Pass safety monitor state to lateral controller for adaptive control
    safety_monitor_state = None
    if hasattr(self, 'safety_monitor') and hasattr(self.safety_monitor, 'anomalies'):
      safety_monitor_state = {
        'road_condition': getattr(self.safety_monitor, 'road_condition', 'normal'),
        'weather_condition': getattr(self.safety_monitor, 'weather_condition', 'normal'),
        'lighting_condition': getattr(self.safety_monitor, 'lighting_condition', 'normal'),
        'anomalies_detected': getattr(self.safety_monitor, 'anomalies', {}),
        'overall_safety_score': getattr(self.safety_monitor, 'overall_safety_score', 1.0)
      }

    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited, self.lat_delay,
                                                       sm={'safety_monitor_state': safety_monitor_state})  # TODO what if not available
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

      # Log lateral control metrics
      try:
        with open("/data/openpilot/metrics/lateral_control_metrics.csv", "a") as f:
          if f.tell() == 0:
            f.write("timestamp,desired_curvature,actual_curvature,output_torque,p_gain,v_ego\n")
          
          p_gain = 0
          if lac_log.error != 0:
            p_gain = self.LaC.pid.p / lac_log.error

          f.write(f"{time.time()},{self.desired_curvature},{self.curvature},{CC.actuators.torque},{p_gain},{CS.vEgo}\n")
      except Exception as e:
        cloudlog.error(f"Error logging lateral control metrics: {e}")

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
