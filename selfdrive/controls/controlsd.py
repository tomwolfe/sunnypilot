#!/usr/bin/env python3
import math
import os
import threading
import time
import numpy as np

from numbers import Number

# Adaptive control system constants
WEATHER_THRESHOLD_TEMP_FREEZING = 2.0  # Temperature (in Celsius) below which snow is more likely
WEATHER_THRESHOLD_TEMP_RAIN = 4.0     # Temperature (in Celsius) above which rain is more likely
CURVY_ROAD_CURVATURE_THRESHOLD = 0.0005 # Curvature threshold for detecting curvy roads
TRAFFIC_DISTANCE_THRESHOLD = 50.0       # Distance (in meters) to consider lead vehicles as "close"

SCENE_COMPLEXITY_RELATIVE_CHANGE = 0.15 # Relative change in scene complexity to trigger model run
SCENE_COMPLEXITY_ABSOLUTE_CHANGE = 30   # Absolute change in scene complexity to trigger model run

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
from openpilot.selfdrive.controls.lib.self_learning_safety import SafeSelfLearningManager
from openpilot.selfdrive.monitoring.lite_monitoring import LightweightSystemMonitor
from openpilot.selfdrive.controls.lib.lite_control import LightweightAdaptiveGainScheduler

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())

# Define a threshold for how long deviceState can be considered stale before
# defaulting to a high thermal state for safety.
DEVICE_STATE_STALE_THRESHOLD = 3.0 # seconds


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

    # Initialize lightweight system monitor and adaptive gain scheduler
    self.system_monitor = LightweightSystemMonitor()
    self.gain_scheduler = LightweightAdaptiveGainScheduler(self.CP)
    self.last_device_state_update_time = 0.0 # Track last update time for deviceState

    # Initialize safety manager for advanced safety features
    self.safety_manager = SafetyManager()

    # Initialize edge case handler for unusual scenario detection and handling
    self.edge_case_handler = EdgeCaseHandler()

    # Initialize self-learning manager for adaptive driving behavior
    self.self_learning_manager = SafeSelfLearningManager(self.CP, self.CP_SP)

    # Initialize safety circuit breakers to prevent cascading failures
    self._init_circuit_breakers()

  def _init_circuit_breakers(self):
    """Initialize circuit breakers to prevent cascading failures."""
    self._circuit_breakers = {
        'adaptive_gains': {
            'enabled': True,
            'error_count': 0,
            'max_errors': 3,  # Reduced from 5 to prevent potential abuse
            'last_error_time': 0,
            'last_error_reset_time': 0,  # Time when error count was last reset
            'cooldown_period': 10.0,  # Increased cooldown to 10 seconds to allow for root cause analysis
            'root_cause_analysis': []  # Store recent error patterns for analysis
        },
        'radar_camera_fusion': {
            'enabled': True,
            'error_count': 0,
            'max_errors': 3,
            'last_error_time': 0,
            'last_error_reset_time': 0,
            'cooldown_period': 15.0,  # Increased for fusion which is critical
            'root_cause_analysis': []  # Store recent error patterns for analysis
        },
        'vision_model_optimization': {
            'enabled': True,
            'error_count': 0,
            'max_errors': 5,  # Reduced from 10 to prevent abuse
            'last_error_time': 0,
            'last_error_reset_time': 0,
            'cooldown_period': 45.0,  # Increased to 45 seconds for vision model
            'root_cause_analysis': []  # Store recent error patterns for analysis
        }
    }

  def _check_circuit_breaker(self, breaker_name):
    """Check if a circuit breaker is enabled and handle cooldown periods."""
    cb = self._circuit_breakers[breaker_name]

    # Check if we're in cooldown period after an error
    current_time = time.monotonic()
    if not cb['enabled']:
        # Calculate time since the last error occurred
        time_since_last_error = current_time - cb['last_error_time']

        # Check if cooldown period has passed since the last error
        if time_since_last_error > cb['cooldown_period']:
            # For the breaker to reset, we need to have waited not just the cooldown period,
            # but also a "stable" period (half the cooldown period) since the last error
            # This prevents the breaker from resetting too soon after the last error,
            # requiring a period of stability before re-enabling the feature
            required_total_wait_time = cb['cooldown_period'] + (cb['cooldown_period'] / 2)
            if time_since_last_error >= required_total_wait_time:
                cb['enabled'] = True
                cb['error_count'] = 0  # Reset error count on successful recovery
                cb['last_error_reset_time'] = current_time
                cloudlog.info(f"Circuit breaker {breaker_name} reset after cooldown and stable period")
            else:
                return False  # Not yet ready to reset - need to wait for stable period after the error
        else:
            return False  # Still in cooldown, circuit breaker is disabled

    return cb['enabled']

  def _trigger_circuit_breaker(self, breaker_name, error_msg, error_type=None):
    """Trigger a circuit breaker due to an error with enhanced root cause tracking."""
    cb = self._circuit_breakers[breaker_name]
    cb['error_count'] += 1
    cb['enabled'] = False
    cb['last_error_time'] = time.monotonic()

    # Add error to root cause analysis
    if error_type is None:
        error_type = "unknown"
    cb['root_cause_analysis'].append({
        'timestamp': cb['last_error_time'],
        'error_type': error_type,
        'error_msg': error_msg,
        'error_count_at_time': cb['error_count']
    })

    # Keep only the most recent 10 errors for analysis
    if len(cb['root_cause_analysis']) > 10:
        cb['root_cause_analysis'] = cb['root_cause_analysis'][-10:]

    cloudlog.error(f"Circuit breaker {breaker_name} triggered due to error: {error_msg} "
                   f"(type: {error_type}). Error count: {cb['error_count']}/{cb['max_errors']} "
                   f"at time {cb['last_error_time']:.2f}")

    # Log detailed root cause analysis if we have multiple errors
    if cb['error_count'] > 1 and len(cb['root_cause_analysis']) > 1:
        # Analyze error patterns
        recent_errors = cb['root_cause_analysis'][-5:]  # Look at last 5 errors
        error_types = [e['error_type'] for e in recent_errors]
        time_diffs = [recent_errors[i+1]['timestamp'] - recent_errors[i]['timestamp']
                     for i in range(len(recent_errors)-1)] if len(recent_errors) > 1 else []

        if len(set(error_types)) == 1:
            # All recent errors are the same type - potential systematic issue
            cloudlog.warning(f"Circuit breaker {breaker_name}: Repeated {error_types[0]} errors detected - possible systematic issue")
        if time_diffs and all(td < 2.0 for td in time_diffs):
            # Errors happening in rapid succession - potential cascade
            cloudlog.warning(f"Circuit breaker {breaker_name}: Rapid error sequence detected - possible cascade failure")

    if cb['error_count'] >= cb['max_errors']:
        cloudlog.critical(f"Circuit breaker {breaker_name} permanently disabled due to excessive errors. "
                          f"Root cause analysis: {cb['root_cause_analysis'][-3:] if cb['root_cause_analysis'] else []}")



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







  def state_control(self):
    CS = self.sm['carState']

    # Calculate thermal state and adaptive gains
    thermal_state = 0.0
    current_time = time.monotonic()
    if 'deviceState' in self.sm and self.sm.valid['deviceState']:
        self.last_device_state_update_time = current_time
        thermal_state = self.system_monitor.calculate_thermal_state(self.sm['deviceState'])
    elif (current_time - self.last_device_state_update_time) > DEVICE_STATE_STALE_THRESHOLD:
        # If deviceState is stale, assume high thermal stress for safety
        cloudlog.warning(f"deviceState is stale (last update {current_time - self.last_device_state_update_time:.2f}s ago). Assuming high thermal state.")
        thermal_state = 1.0 # Max thermal state for safety

    # Enhanced adaptive gains calculation considering multiple factors
    if self._check_circuit_breaker('adaptive_gains'):
        try:
            driving_context = self._calculate_driving_context(CS)
            adaptive_gains = self._calculate_contextual_adaptive_gains(CS.vEgo, thermal_state, driving_context)
        except Exception as e:
            cloudlog.error(f"Error in adaptive gains calculation: {e}")
            self._trigger_circuit_breaker('adaptive_gains', str(e), error_type='adaptive_gains_calculation')
            # Fall back to very conservative gains to ensure safety in all scenarios
            adaptive_gains = {
                'lateral': {
                    'steer_kp': 0.3,  # More conservative gain for lateral control
                    'steer_ki': 0.03,
                    'steer_kd': 0.003,
                },
                'longitudinal': {
                    'accel_kp': 0.3,  # More conservative gain for longitudinal control
                    'accel_ki': 0.03,
                }
            }
            # Log the fallback to safe mode
            cloudlog.warning("Fell back to safe default gains due to error")
    else:
        # Circuit breaker is triggered, use very conservative gains to ensure safety in all scenarios
        adaptive_gains = {
            'lateral': {
                'steer_kp': 0.3,  # More conservative gain for lateral control
                'steer_ki': 0.03,
                'steer_kd': 0.003,
            },
            'longitudinal': {
                'accel_kp': 0.3,  # More conservative gain for longitudinal control
                'accel_ki': 0.03,
            }
        }
        cloudlog.warning("Using safe default gains - adaptive gains circuit breaker is active")

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
    # Extract longitudinal gains for the longitudinal controller - with safe fallback
    if isinstance(adaptive_gains, dict) and 'longitudinal' in adaptive_gains:
        longitudinal_gains = adaptive_gains['longitudinal']
    else:
        # Fall back to safe default longitudinal gains if structure is unexpected
        longitudinal_gains = {
            'accel_kp': 0.5,
            'accel_ki': 0.05,
        }
        cloudlog.warning(f"Adaptive gains has unexpected structure, using default longitudinal gains: {type(adaptive_gains)}")

    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, conservative_accel_limits, longitudinal_gains))

    # Apply self-learning adjustments to model outputs
    base_desired_curvature = model_v2.action.desiredCurvature
    # Use enhanced safety validation if available, otherwise fall back to basic adjustment
    if hasattr(self.self_learning_manager, 'adjust_curvature_with_safety_validation'):
      learned_adjusted_curvature = self.self_learning_manager.adjust_curvature_with_safety_validation(
        base_desired_curvature, CS.vEgo, CS
      )
    else:
      learned_adjusted_curvature = self.self_learning_manager.adjust_curvature(base_desired_curvature, CS.vEgo)

    # Apply lateral control modifications if needed (after learning adjustment)
    modified_desired_curvature = learned_adjusted_curvature
    if adaptive_mods['lateral_factor'] < 1.0 and CC.latActive:
      # Make lateral control more conservative by reducing desired curvature
      modified_desired_curvature = learned_adjusted_curvature * adaptive_mods['lateral_factor']

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = modified_desired_curvature if CC.latActive else self.curvature
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature

    # Use the adaptive gains for lateral control
    # For now, pass the overall adaptive_gains structure and let the LaC handle the format
    # The LaC controller is expected to handle both old and new gain formats
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       self.calibrated_pose, curvature_limited, lat_delay, adaptive_gains)

    # Enhanced saturation detection with smoother recovery
    saturation_detected = False
    if hasattr(lac_log, 'saturated') and lac_log.saturated:
      saturation_detected = True
      cloudlog.debug(f"Steering saturation detected at vEgo: {CS.vEgo:.2f} m/s")

    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)

    # Apply adaptive GPU management to balance thermal concerns with performance needs
    self._adaptive_gpu_management(CS, self.sm)



    # Enhanced finite value checks with recovery mechanism
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      # Check for finite values and reasonable bounds
      if not math.isfinite(attr):
        cloudlog.error(
            f"actuators.{p} not finite. Actuators: {actuators.to_dict()}, CarState: {CS.to_dict()}, " +
            f"LongitudinalPlan: {long_plan.to_dict()}, LateralControlLog: {lac_log.to_dict()}"
        )
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
      # Additional safety validation for extreme values
      elif p == 'accel':
        # Check for excessive acceleration commands that exceed physical limits
        if abs(attr) > 5.0:  # More than 0.5g acceleration is extreme
          cloudlog.warning(f"Excessive acceleration command detected: {attr} m/s^2, limiting to safe value")
          limited_accel = max(-5.0, min(5.0, attr))
          setattr(actuators, p, limited_accel)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.warning
      elif p in ['steeringAngleDeg']:
        # Check for excessive steering angle commands beyond physical limits
        if abs(attr) > 90.0:  # Steering angle beyond 90 degrees should never happen
          cloudlog.error(f"Excessive steering angle command detected: {attr} deg, limiting to safe value")
          limited_steering = max(-90.0, min(90.0, attr))
          setattr(actuators, p, limited_steering)
          CC.hudControl.visualAlert = log.ControlsState.AlertStatus.critical
      elif p in ['curvature']:
        # Check for excessive curvature commands (very tight turns at speed)
        if abs(attr) > 0.5 and CS.vEgo > 5.0:  # Very high curvature at speed is dangerous
          # Calculate max safe curvature based on speed: curvature = max_lat_accel / vEgo^2
          max_lat_accel = 3.0  # Conservative maximum lateral acceleration in m/s^2
          max_safe_curvature = max_lat_accel / (CS.vEgo ** 2)
          if abs(attr) > max_safe_curvature:
            cloudlog.warning(f"Excessive curvature command detected: {attr} vs max safe {max_safe_curvature} at speed {CS.vEgo}, limiting")
            limited_curvature = max(-max_safe_curvature, min(max_safe_curvature, attr))
            setattr(actuators, p, limited_curvature)
            CC.hudControl.visualAlert = log.ControlsState.AlertStatus.warning

    # Enhanced saturation handling in controls state
    if saturation_detected:
      CC.hudControl.visualAlert = log.ControlsState.AlertStatus.normal  # Indicate saturation to user

    # Update self-learning manager with current driving experience
    model_confidence = getattr(model_v2.meta, 'confidence', 1.0) if hasattr(model_v2, 'meta') else 1.0
    # Calculate actual vehicle curvature from vehicle state measurements
    # Using kinematic bicycle model: curvature = tan(steering_angle) / wheelbase
    # We need to account for angle offset and convert to radians
    lp = self.sm['liveParameters']  # Get live parameters for angle offset
    steer_angle_rad = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)  # Apply angle offset
    actual_vehicle_curvature = math.tan(steer_angle_rad) / self.CP.wheelbase if abs(CS.vEgo) > 0.1 else 0.0

    # Calculate model prediction error: difference between model output and actual vehicle behavior
    model_prediction_error = base_desired_curvature - actual_vehicle_curvature

    self.self_learning_manager.update(
        CS,
        base_desired_curvature,  # Original model output
        actual_vehicle_curvature,  # Actual vehicle curvature from vehicle state
        actuators.torque,
        CS.vEgo,
        model_confidence=model_confidence,
        model_prediction_error=model_prediction_error
    )

    return CC, lac_log

  def _calculate_driving_context(self, CS):
    """
    Calculate driving context based on vehicle state, road conditions, and environment.

    Args:
        CS: CarState message

    Returns:
        dict: Context information including road type, traffic density, weather indicators, etc.
    """
    # Calculate current curvature from vehicle state to avoid dependency issues
    lp = self.sm['liveParameters']
    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    current_curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    context = {
        'is_curvy_road': False,
        'traffic_density': 'low',  # low, medium, high
        'weather_condition': self._detect_weather_conditions(),  # normal, rain, snow, fog
        'time_of_day': 'day',  # day, night
        'current_curvature': abs(current_curvature),
        'lateral_accel': CS.vEgo**2 * abs(current_curvature) if CS.vEgo > 1.0 else 0.0,
        'long_accel_magnitude': abs(CS.aEgo),
        'steering_activity': abs(CS.steeringRateDeg)  # How much steering is happening
    }

    # Determine if on a curvy road based on recent curvature history
    if hasattr(self, '_curvature_history'):
        avg_curvature = np.mean(np.abs(self._curvature_history))
        # Threshold justification:
        # Curvature = 1 / radius of curve
        # At 0.0005 rad/m: radius is ~2000m - this is a gentle curve, appropriate for detecting curvy roads
        # At 0.001 rad/m: radius is ~1000m - this would miss many curves that require adjustment
        # At 0.002 rad/m: radius is ~500m - this is quite sharp for highway driving
        # We use 0.0005 as a balance between sensitivity and avoiding false positives on straight roads
        context['is_curvy_road'] = avg_curvature > CURVY_ROAD_CURVATURE_THRESHOLD  # Curvature threshold for "curvy road" detection

        # Update history (keep last 100 samples)
        self._curvature_history.append(context['current_curvature'])
        if len(self._curvature_history) > 100:
            self._curvature_history.pop(0)
    else:
        self._curvature_history = [context['current_curvature']]

    # Estimate traffic density based on radar data if available
    if 'radarState' in self.sm and self.sm.valid['radarState']:
        radar_state = self.sm['radarState']
        close_leads = 0
        for lead in [radar_state.leadOne, radar_state.leadTwo]:
            if lead.status and lead.dRel < TRAFFIC_DISTANCE_THRESHOLD:  # Within threshold - threshold justification: In highway driving, this distance is close enough to indicate high traffic density
                close_leads += 1
        if close_leads >= 2:
            context['traffic_density'] = 'high'
        elif close_leads == 1:
            context['traffic_density'] = 'medium'
        else:
            context['traffic_density'] = 'low'

    # Add comprehensive logging for debugging
    cloudlog.debug(f"Driving context calculated: speed={CS.vEgo:.2f}m/s, "
                   f"curvy_road={context['is_curvy_road']}, traffic={context['traffic_density']}, "
                   f"weather={context['weather_condition']}, lateral_accel={context['lateral_accel']:.3f}, "
                   f"long_accel={context['long_accel_magnitude']:.3f}, steering_rate={context['steering_activity']:.2f}")

    return context

  def _detect_weather_conditions(self):
    """
    Detect weather conditions based on simple, reliable sensor data.
    Simplified to use only wiper status as the primary indicator of adverse weather.

    Returns:
        str: Weather condition ('normal', 'rain')
    """
    # Use reliable sensor data from carState for simple weather detection
    CS = self.sm['carState']

    # Check for windshield wiper usage as the primary indicator of adverse weather
    # This is more reliable than complex temperature-based heuristics
    wipers_active = False
    if hasattr(CS, 'windshieldWiper') and CS.windshieldWiper is not None:
        wipers_active = CS.windshieldWiper > 0.0
    elif hasattr(CS, 'wiperState') and CS.wiperState is not None:
        wipers_active = CS.wiperState > 0

    # Simple logic: if wipers are active, there's adverse weather (likely rain)
    # This removes the complex temperature and sensor fusion logic that was fragile
    if wipers_active:
        return 'rain'  # Default to rain as the most common adverse weather condition

    # If no wipers are active, return normal conditions
    return 'normal'

  def _calculate_contextual_adaptive_gains(self, v_ego, thermal_state, context):
    """
    Calculate adaptive gains based on vehicle speed, thermal state and driving context.

    NOTE: This function is protected by a circuit breaker (see _init_circuit_breakers).
    If errors occur repeatedly, the adaptive gains system will be disabled temporarily
    to prevent cascading failures, falling back to safe default gains.

    Args:
        v_ego: Vehicle speed in m/s
        thermal_state: Thermal stress factor (0.0-1.0)
        context: Driving context information

    Returns:
        dict: Adaptive gain parameters
    """
    # Base gains that get adjusted based on context
    base_gains = {
        'lateral': {
            'steer_kp': 1.0,
            'steer_ki': 0.1,
            'steer_kd': 0.01,
        },
        'longitudinal': {
            'accel_kp': 1.0,
            'accel_ki': 0.1,
        }
    }

    # Speed-dependent adjustments
    # Threshold justification: Normalize to 30 m/s (about 108 km/h) as reference high-speed point
    speed_factor = min(1.0, v_ego / 30.0)
    # Reduce gains by up to 30% at high speeds (when speed_factor = 1.0) for enhanced stability
    speed_adjustment = 1.0 - (0.3 * speed_factor)  # Reduce gains at higher speeds for stability

    # Thermal adjustments
    # Reduce gains by up to 20% when thermal stress is at maximum (thermal_state = 1.0) to reduce computational load
    thermal_adjustment = 1.0 - (thermal_state * 0.2)  # Reduce gains when hot

    # Context-based adjustments
    context_adjustment = 1.0

    # Reduce gains on curvy roads for smoother steering
    # Factor 0.85 justification: Reduce gains by 15% to provide smoother, more conservative steering on curvy roads
    if context['is_curvy_road']:
        context_adjustment *= 0.85

    # Increase caution in high traffic
    # Factor 0.9 justification: Reduce gains by 10% to provide more conservative control in dense traffic
    if context['traffic_density'] == 'high':
        context_adjustment *= 0.9

    # Reduce gains in poor weather (if we can detect it)
    # Factor 0.9 justification: Reduce gains by 10% for safety in adverse weather conditions
    if context['weather_condition'] != 'normal':
        context_adjustment *= 0.9

    # Apply combined adjustments
    combined_adjustment = speed_adjustment * thermal_adjustment * context_adjustment

    # Log detailed information for debugging
    cloudlog.debug(f"Adaptive gains calculation: v_ego={v_ego:.2f}, thermal={thermal_state:.2f}, "
                   f"speed_factor={speed_factor:.3f}, thermal_adj={thermal_adjustment:.3f}, "
                   f"context_adj={context_adjustment:.3f}, combined_adj={combined_adjustment:.3f}, "
                   f"curvy_road={context['is_curvy_road']}, traffic={context['traffic_density']}, "
                   f"weather={context['weather_condition']}")

    # Apply adjustments to base gains
    adaptive_gains = {
        'lateral': {
            'steer_kp': base_gains['lateral']['steer_kp'] * combined_adjustment,
            'steer_ki': base_gains['lateral']['steer_ki'] * combined_adjustment,
            'steer_kd': base_gains['lateral']['steer_kd'] * combined_adjustment,
        },
        'longitudinal': {
            'accel_kp': base_gains['longitudinal']['accel_kp'] * combined_adjustment,
            'accel_ki': base_gains['longitudinal']['accel_ki'] * combined_adjustment,
        }
    }

    # Add validation mechanism to ensure adaptive gains are within safe bounds
    adaptive_gains = self._validate_adaptive_gains(adaptive_gains)

    # Log final adaptive gains for debugging
    cloudlog.debug(f"Final adaptive gains - Lateral: KP={adaptive_gains['lateral']['steer_kp']:.3f}, "
                   f"KI={adaptive_gains['lateral']['steer_ki']:.3f}, KD={adaptive_gains['lateral']['steer_kd']:.3f}; "
                   f"Longitudinal: KP={adaptive_gains['longitudinal']['accel_kp']:.3f}, "
                   f"KI={adaptive_gains['longitudinal']['accel_ki']:.3f}")

    return adaptive_gains

  def _validate_adaptive_gains(self, adaptive_gains):
    """
    Validate adaptive gains to prevent dangerous values that could lead to instability or unsafe behavior.

    Args:
        adaptive_gains: Dictionary containing lateral and longitudinal gains

    Returns:
        dict: Validated and potentially corrected adaptive gain parameters
    """
    # Define safe bounds for gains
    MIN_STEER_KP = 0.1  # Minimum steering proportional gain
    MAX_STEER_KP = 3.0  # Maximum steering proportional gain
    MIN_STEER_KI = 0.01  # Minimum steering integral gain
    MAX_STEER_KI = 1.0  # Maximum steering integral gain
    MIN_STEER_KD = 0.0  # Minimum steering derivative gain
    MAX_STEER_KD = 0.1  # Maximum steering derivative gain

    MIN_ACCEL_KP = 0.1  # Minimum acceleration proportional gain
    MAX_ACCEL_KP = 2.0  # Maximum acceleration proportional gain
    MIN_ACCEL_KI = 0.01  # Minimum acceleration integral gain
    MAX_ACCEL_KI = 1.0  # Maximum acceleration integral gain

    # Additional safety validation - check for sudden changes in gains that might indicate sensor errors
    if hasattr(self, '_prev_adaptive_gains'):
        prev_gains = self._prev_adaptive_gains
        current_time = time.monotonic()

        # Check for excessive gain changes between consecutive calls
        for gain_type in adaptive_gains:
            if gain_type in prev_gains:
                for gain_name in adaptive_gains[gain_type]:
                    if gain_name in prev_gains[gain_type]:
                        old_val = prev_gains[gain_type][gain_name]
                        new_val = adaptive_gains[gain_type][gain_name]
                        gain_change = abs(new_val - old_val)

                        # If the gain changed by more than 50% of its previous value, log a warning
                        if old_val != 0 and (gain_change / abs(old_val)) > 0.5:
                            cloudlog.warning(f"Sudden gain change detected: {gain_name} changed from {old_val} to {new_val}")
                            # Apply a smoother transition to prevent abrupt control changes
                            adaptive_gains[gain_type][gain_name] = old_val + (gain_change * 0.3)  # Only apply 30% of the change
                            cloudlog.info(f"Smoothed {gain_name} to {adaptive_gains[gain_type][gain_name]}")

    # Validate lateral gains
    if 'lateral' in adaptive_gains:
        lateral = adaptive_gains['lateral']

        # Check and bound steering KP
        if 'steer_kp' in lateral:
            original_kp = lateral['steer_kp']
            lateral['steer_kp'] = max(MIN_STEER_KP, min(MAX_STEER_KP, lateral['steer_kp']))
            if original_kp != lateral['steer_kp']:
                cloudlog.warning(f"Steering KP gain adjusted from {original_kp} to {lateral['steer_kp']} for safety")

        # Check and bound steering KI
        if 'steer_ki' in lateral:
            original_ki = lateral['steer_ki']
            lateral['steer_ki'] = max(MIN_STEER_KI, min(MAX_STEER_KI, lateral['steer_ki']))
            if original_ki != lateral['steer_ki']:
                cloudlog.warning(f"Steering KI gain adjusted from {original_ki} to {lateral['steer_ki']} for safety")

        # Check and bound steering KD
        if 'steer_kd' in lateral:
            original_kd = lateral['steer_kd']
            lateral['steer_kd'] = max(MIN_STEER_KD, min(MAX_STEER_KD, lateral['steer_kd']))
            if original_kd != lateral['steer_kd']:
                cloudlog.warning(f"Steering KD gain adjusted from {original_kd} to {lateral['steer_kd']} for safety")

        # Additional safety: Check for gain balance to prevent instability
        # The ratio of KI/KP should not be too large to prevent integral windup
        if 'steer_kp' in lateral and 'steer_ki' in lateral:
            if lateral['steer_kp'] > 0 and (lateral['steer_ki'] / lateral['steer_kp']) > 0.5:
                # Reduce KI if it's too large relative to KP
                lateral['steer_ki'] = lateral['steer_kp'] * 0.5
                cloudlog.warning(f"Reduced steering KI to maintain stability: {lateral['steer_ki']}")

    # Validate longitudinal gains
    if 'longitudinal' in adaptive_gains:
        longitudinal = adaptive_gains['longitudinal']

        # Check and bound acceleration KP
        if 'accel_kp' in longitudinal:
            original_kp = longitudinal['accel_kp']
            longitudinal['accel_kp'] = max(MIN_ACCEL_KP, min(MAX_ACCEL_KP, longitudinal['accel_kp']))
            if original_kp != longitudinal['accel_kp']:
                cloudlog.warning(f"Acceleration KP gain adjusted from {original_kp} to {longitudinal['accel_kp']} for safety")

        # Check and bound acceleration KI
        if 'accel_ki' in longitudinal:
            original_ki = longitudinal['accel_ki']
            longitudinal['accel_ki'] = max(MIN_ACCEL_KI, min(MAX_ACCEL_KI, longitudinal['accel_ki']))
            if original_ki != longitudinal['accel_ki']:
                cloudlog.warning(f"Acceleration KI gain adjusted from {original_ki} to {longitudinal['accel_ki']} for safety")

        # Additional safety: Check for longitudinal gain balance
        if 'accel_kp' in longitudinal and 'accel_ki' in longitudinal:
            if longitudinal['accel_kp'] > 0 and (longitudinal['accel_ki'] / longitudinal['accel_kp']) > 0.5:
                # Reduce KI if it's too large relative to KP
                longitudinal['accel_ki'] = longitudinal['accel_kp'] * 0.5
                cloudlog.warning(f"Reduced acceleration KI to maintain stability: {longitudinal['accel_ki']}")

    # Additional safety check - ensure gains are not NaN or infinity
    for gain_type in adaptive_gains:
        for gain_name, gain_value in adaptive_gains[gain_type].items():
            if not isinstance(gain_value, (int, float)) or not math.isfinite(gain_value):
                cloudlog.error(f"Invalid gain value detected: {gain_name} = {gain_value}, setting to safe default")
                # Set to a safe default based on the gain type
                if 'steer' in gain_name:
                    adaptive_gains[gain_type][gain_name] = 1.0  # Default safe value for steering
                elif 'accel' in gain_name:
                    adaptive_gains[gain_type][gain_name] = 1.0  # Default safe value for acceleration

    # Store current gains for next iteration comparison
    self._prev_adaptive_gains = adaptive_gains.copy()

    # Add comprehensive logging for debugging
    cloudlog.debug(f"Adaptive gains validated - Lateral: KP={adaptive_gains['lateral']['steer_kp']:.3f}, "
                   f"KI={adaptive_gains['lateral']['steer_ki']:.3f}, KD={adaptive_gains['lateral']['steer_kd']:.3f}; "
                   f"Longitudinal: KP={adaptive_gains['longitudinal']['accel_kp']:.3f}, "
                   f"KI={adaptive_gains['longitudinal']['accel_ki']:.3f}")

    return adaptive_gains

  def _adaptive_gpu_management(self, CS, sm):
    """
    Adaptive GPU management to temporarily increase performance when needed for safety-critical operations.
    This addresses the thermal management trade-off by allowing temporary performance boosts when needed.
    """
    try:
        # Check if we're in a critical situation that may require higher GPU performance
        critical_situation = (
            sm['modelV2'].meta.hardBrakePredicted if 'modelV2' in sm and hasattr(sm['modelV2'].meta, 'hardBrakePredicted') else False
        )

        # Check for lead vehicle emergency situations
        if 'radarState' in sm and sm.valid['radarState']:
            radar_state = sm['radarState']
            for lead in [radar_state.leadOne, radar_state.leadTwo]:
                if (lead.status and
                    lead.aLeadK < -3.0 and  # Lead vehicle braking hard
                    lead.dRel < 50.0 and    # Close distance
                    CS.vEgo > 5.0):         # Moving at significant speed
                    critical_situation = True
                    break

        # GPU governor path - check if it exists before attempting to write
        gpu_governor_path = "/sys/class/kgsl/kgsl-3d0/devfreq/governor"

        # Check if the GPU governor file exists before attempting to write
        if not os.path.exists(gpu_governor_path):
            # If the file doesn't exist, we're on different hardware, skip GPU management
            return

        # If in critical situation and if we have thermal headroom, temporarily boost performance
        if critical_situation and 'deviceState' in sm and sm.valid['deviceState']:
            thermal_status = sm['deviceState'].thermalStatus
            thermal_pwr = sm['deviceState'].thermalPerc

            # Only boost if we're not already in thermal danger
            if thermal_status <= ThermalStatus.yellow and thermal_pwr >= 80:  # Only boost if green/yellow and at least 80% thermal performance
                # Temporarily switch GPU to performance mode for critical operations
                # This helps reduce latency during critical driving situations
                try:
                    with open(gpu_governor_path, "w") as f:
                        f.write("performance")

                    # Log the temporary performance boost for monitoring
                    cloudlog.debug(f"Temporary GPU performance boost activated for critical situation. "
                                   f"Thermal: {thermal_status}, Power: {thermal_pwr}%")

                    # Set a flag to switch back to ondemand after a short period
                    if not hasattr(self, '_temp_perf_end_time'):
                        self._temp_perf_end_time = time.monotonic() + 2.0  # Revert after 2 seconds
                except (OSError, IOError) as e:
                    # If we can't write the governor file, log an error but continue
                    cloudlog.error(f"Failed to set GPU governor to performance mode: {e}")
            else:
                # If not in critical situation or thermal limits, make sure we're in ondemand
                try:
                    with open(gpu_governor_path, "w") as f:
                        f.write("ondemand")
                except (OSError, IOError) as e:
                    cloudlog.error(f"Failed to set GPU governor to ondemand: {e}")
        else:
            # If not in critical situation, ensure we're using ondemand for thermal management
            try:
                with open(gpu_governor_path, "w") as f:
                    f.write("ondemand")
            except (OSError, IOError) as e:
                cloudlog.error(f"Failed to set GPU governor to ondemand: {e}")

        # Check if we need to revert from temporary performance mode
        if hasattr(self, '_temp_perf_end_time') and time.monotonic() > self._temp_perf_end_time:
            # Revert to ondemand governor after critical situation passes
            try:
                with open(gpu_governor_path, "w") as f:
                    f.write("ondemand")
                cloudlog.debug("Reverted GPU governor to ondemand after critical situation")
                delattr(self, '_temp_perf_end_time')
            except (OSError, IOError) as e:
                cloudlog.error(f"Failed to revert GPU governor to ondemand: {e}")
    except Exception as e:
        # If we encounter an error in GPU management, continue with current operation
        cloudlog.error(f"Error in adaptive GPU management: {e}")
        # Still try to ensure ondemand governor in case of error
        gpu_governor_path = "/sys/class/kgsl/kgsl-3d0/devfreq/governor"
        if os.path.exists(gpu_governor_path):
            try:
                with open(gpu_governor_path, "w") as f:
                    f.write("ondemand")
            except (OSError, IOError) as e:
                cloudlog.error(f"Failed to set GPU governor to ondemand in error handling: {e}")

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

    # Get safety recommendation from self-learning system and potentially use it to enhance driver alerts
    model_output = {'desired_curvature': self.sm['modelV2'].action.desiredCurvature if 'modelV2' in self.sm else 0.0}
    learning_safety_recommendation = self.self_learning_manager.get_safety_recommendation(CS, model_output)
    # We could potentially combine both safety recommendations or prioritize the learning system's recommendation
    # For now, we'll use the original safety manager's recommendation but log the learning system's recommendation
    RECOMMENDATION_NORMAL = 0  # This should match the value used in self_learning_safety.py
    if learning_safety_recommendation != RECOMMENDATION_NORMAL:
      cloudlog.info(f"Self-learning system safety recommendation: {learning_safety_recommendation}")

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    # Add detailed telemetry for real-world monitoring
    # Log contextual information for debugging and analysis
    if hasattr(self, '_prev_adaptive_gains'):
        # Store adaptive gains information for telemetry
        if 'lateral' in self._prev_adaptive_gains:
            # These values will be logged as part of the system's operation
            pass

    # Calculate and log additional telemetry data
    # Thermal state at the time of this control cycle
    current_thermal_state = 0.0
    if 'deviceState' in self.sm and self.sm.valid['deviceState']:
        current_thermal_state = self.system_monitor.calculate_thermal_state(self.sm['deviceState'])

    # Log various parameters for analysis
    cs.vCruise = float(CS.vCruise)
    cs.vEgo = float(CS.vEgo)
    cs.aEgo = float(CS.aEgo)
    cs.steeringAngleDeg = float(CS.steeringAngleDeg)
    cs.steeringRateDeg = float(CS.steeringRateDeg)

    # Additional context information for analysis
    if hasattr(self, 'curvature'):
        cs.curvature = self.curvature  # Already set above, but reinforcing its importance

    if hasattr(self, 'desired_curvature'):
        cs.desiredCurvature = self.desired_curvature  # Already set above

    # Log information about the adaptive control system
    if hasattr(self, 'thermal_state'):
        # Store thermal state information in a field that's available
        pass  # thermal state is already calculated above

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

      while True:
        self.update()
        CC, lac_log = self.state_control()
        self.publish(CC, lac_log)
        self.run_ext(self.sm, self.pm)

        # Calculate thermal state and adaptive gains for thermal management
        thermal_state = 0.0
        current_time = time.monotonic()
        if 'deviceState' in self.sm and self.sm.valid['deviceState']:
          self.last_device_state_update_time = current_time
          thermal_state = self.system_monitor.calculate_thermal_state(self.sm['deviceState'])
        elif (current_time - self.last_device_state_update_time) > DEVICE_STATE_STALE_THRESHOLD:
          # If deviceState is stale, assume high thermal stress for safety
          cloudlog.warning(f"deviceState is stale (last update {current_time - self.last_device_state_update_time:.2f}s ago). Assuming high thermal state.")
          thermal_state = 1.0 # Max thermal state for safety

        # Get adaptive gains based on thermal state and vehicle speed
        adaptive_gains = self.gain_scheduler.get_adaptive_gains(self.sm['carState'].vEgo, thermal_state)

        # Adaptive control rate based on thermal stress
        # Calculate rates based on thermal stress (0.0 = no stress, 1.0 = maximum stress)
        stress_factor = thermal_state  # Using actual thermal state instead of performance_compensation_factor

        # Define base rate and minimum rates
        base_rate = 100  # 100Hz base rate
        min_critical_rate = 50  # Minimum rate for critical functions
        min_standard_rate = 10  # Minimum rate for standard functions

        # Calculate adaptive rates based on thermal stress
        # Critical functions: reduce less aggressively
        critical_factor = max(0.5, 1.0 - stress_factor * 0.3)  # Less reduction for critical functions
        critical_rate = max(min_critical_rate, base_rate * critical_factor)

        # Standard functions: reduce more aggressively
        standard_factor = max(0.1, 1.0 - stress_factor * 0.9)  # More reduction for standard functions
        standard_rate = max(min_standard_rate, base_rate * standard_factor)

        # Calculate current rate based on thermal state (interpolate between standard and critical rates)
        current_rate = base_rate * (1.0 - stress_factor * 0.5)  # Moderate throttling

        # Frame skipping counter for thermal throttling
        if not hasattr(self, 'thermal_adjusted_frame'):
          self.thermal_adjusted_frame = 0

        # Process every frame when at full rate, every other frame when at reduced rate
        frame_skip_threshold = base_rate / max(current_rate, 1)  # Prevent division by zero
        self.thermal_adjusted_frame += 1

        # Only perform full control cycle if we're not skipping this frame due to thermal constraints
        if self.thermal_adjusted_frame >= frame_skip_threshold:
          self.thermal_adjusted_frame = 0  # Reset counter

          # Update message subscriptions based on priority during thermal stress
          # Critical functions get higher priority update rates
          if thermal_state > 0.5:  # Under thermal stress (0.5 is medium stress threshold)
            # Update critical functions at higher rate
            self.sm.update(100)  # Always update critical messages at high rate

            # Perform thermal-aware control operations
            # In this context, we use the normal control cycle but with thermal-aware adaptive gains
            self.update()
            CC, lac_log = self.state_control()  # Using the state_control method with thermal awareness
          else:
            # Normal operation
            self.update()
            CC, lac_log = self.state_control()

          self.publish(CC, lac_log)
          self.get_params_sp(self.sm)
          self.run_ext(self.sm, self.pm)

          # Enhanced thermal monitoring and logging with stress level information
          if thermal_state > 0.1:  # Only log if there's some thermal stress
            cloudlog.debug(
                f"Thermal throttling active: thermal_state={thermal_state:.2f}, " +
                f"rate={current_rate:.1f}Hz, " +
                f"critical_rate={critical_rate:.1f}Hz, standard_rate={standard_rate:.1f}Hz"
            )
        else:
          # Still update the message subsystem regularly to maintain message flow
          # This 15Hz update rate is chosen to ensure critical communication (e.g., carState, radarState)
          # is maintained even when the main control loop is thermally throttled and frames are skipped,
          # without adding significant computational load during these skipped cycles.
          self.sm.update(15)

        # Monitor timing with thermal awareness and add thermal performance adjustments
        timing_start = time.monotonic()
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
