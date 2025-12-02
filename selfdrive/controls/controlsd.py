#!/usr/bin/env python3
import math
import threading
import time
import numpy as np

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
    driving_context = self._calculate_driving_context(CS)
    adaptive_gains = self._calculate_contextual_adaptive_gains(CS.vEgo, thermal_state, driving_context)

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
    # NOTE: The adaptive_gains dictionary includes both 'accel_kp' and 'accel_ki' values
    # The longitudinal controller (self.LoC) should properly use these PID gains for better control
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, conservative_accel_limits, adaptive_gains))

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

    # Use the adaptive gains for lateral control - pass more detailed gain information if available
    if isinstance(adaptive_gains, dict) and 'lateral' in adaptive_gains:
        # Pass specific lateral adaptive gains to the controller
        lateral_gains = adaptive_gains['lateral']
        steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                           self.steer_limited_by_safety, self.desired_curvature,
                                                           self.calibrated_pose, curvature_limited, lat_delay, lateral_gains)
    else:
        # Fall back to original approach
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
    context = {
        'is_curvy_road': False,
        'traffic_density': 'low',  # low, medium, high
        'weather_condition': self._detect_weather_conditions(),  # normal, rain, snow, fog
        'time_of_day': 'day',  # day, night
        'current_curvature': abs(self.curvature),
        'lateral_accel': CS.vEgo**2 * abs(self.curvature) if CS.vEgo > 1.0 else 0.0,
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
        context['is_curvy_road'] = avg_curvature > 0.0005  # Curvature threshold for "curvy road" detection

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
            if lead.status and lead.dRel < 50.0:  # Within 50m - threshold justification: In highway driving, 50m is close enough to indicate high traffic density
                close_leads += 1
        if close_leads >= 2:
            context['traffic_density'] = 'high'
        elif close_leads == 1:
            context['traffic_density'] = 'medium'
        else:
            context['traffic_density'] = 'low'

    return context

  def _detect_weather_conditions(self):
    """
    Detect weather conditions based on available and reliable sensor data.
    Enhanced to properly differentiate between rain and snow using temperature data.

    Returns:
        str: Weather condition ('normal', 'rain', 'snow', 'fog')
    """
    # Use reliable sensor data from carState and modelV2 for comprehensive weather detection
    CS = self.sm['carState']

    # 1. Check for windshield wiper usage as an indicator of precipitation
    wipers_active = (hasattr(CS, 'windshieldWiper') and CS.windshieldWiper > 0.0) or \
                    (hasattr(CS, 'wiperState') and CS.wiperState > 0)

    # 2. Get temperature data if available to differentiate between rain and snow
    # Many vehicle systems provide ambient temperature data
    ambient_temp = None
    if hasattr(CS, 'ambientTemp') and CS.ambientTemp is not None:
        ambient_temp = CS.ambientTemp
    elif hasattr(CS, 'carParams') and hasattr(self.CP, 'carParams') and hasattr(self.CP.carParams, 'ambientTemp'):
        # If ambient temperature is available in car params
        ambient_temp = self.CP.carParams.ambientTemp

    # If ambient temperature is not available in CarState, try to get it from other sources
    if 'carParams' in self.sm and hasattr(self.sm['carParams'], 'ambientTemp'):
        ambient_temp = self.sm['carParams'].ambientTemp

    # 3. Check for additional indicators to differentiate between weather types
    if 'modelV2' in self.sm and self.sm.valid['modelV2'] and self.sm.valid['radarState']:
        model_v2 = self.sm['modelV2']
        radar_state = self.sm['radarState']

        # Analyze scene characteristics for different weather types
        avg_lane_prob = 0.0
        if (hasattr(model_v2, 'laneLines') and len(model_v2.laneLines) > 0 and
            hasattr(model_v2.laneLines[0], 'prob')):
            avg_lane_prob = sum([line.prob for line in model_v2.laneLines]) / len(model_v2.laneLines)

        # Additional indicators for fog detection
        fog_indicators = 0
        snow_indicators = 0

        # Fog: low visibility + low lane line confidence + radar-camera discrepancy
        if (hasattr(model_v2, 'meta') and hasattr(model_v2.meta, 'lowVis') and model_v2.meta.lowVis) or \
           (avg_lane_prob < 0.5 and radar_state.leadOne.status and radar_state.leadOne.dRel > 50.0) or \
           (hasattr(model_v2, 'leadsV3') and
            sum(1 for lead in model_v2.leadsV3 if lead.prob > 0.5) == 0 and
            any(lead.status for lead in [radar_state.leadOne, radar_state.leadTwo])):
            fog_indicators += 1

        # Snow indicators: poor lane detection combined with temperature data
        # If we have temperature data and it's below freezing, and we see poor lane detection, likely snow
        if ambient_temp is not None and ambient_temp < 2.0:  # Below 2°C (slightly above freezing to account for measurement errors)
            if avg_lane_prob < 0.5:
                snow_indicators += 1
            # Snow often results in more cautious driving behavior
            if CS.vEgo < 20.0 and avg_lane_prob < 0.7:  # Lower speeds with reduced lane detection confidence
                snow_indicators += 1

        # Check for headlight usage as ancillary indicator
        headlights_on_in_daytime = (hasattr(CS, 'lowBeam') and CS.lowBeam and
                                    hasattr(CS, 'solarRad') and CS.solarRad > 10000)

        # If we have temperature data available
        if ambient_temp is not None:
            if ambient_temp > 4.0 and wipers_active and fog_indicators == 0:
                # Above freezing, wipers on but no fog indicators - likely rain
                return 'rain'
            elif ambient_temp <= 2.0 and wipers_active:
                # Below freezing and wipers on - likely snow (especially if snow indicators present)
                if snow_indicators > 0:
                    return 'snow'
                else:
                    # If temperature is below freezing but no clear snow indicators,
                    # it might be snow or a mix; default to snow when temp is low
                    return 'snow'
            elif ambient_temp <= 4.0 and ambient_temp > 2.0 and wipers_active:
                # Between 2-4°C - could be either rain or snow depending on road conditions
                # Use additional indicators to make the decision
                if snow_indicators > fog_indicators:  # More snow indicators than fog indicators
                    return 'snow'
                else:
                    return 'rain'
            elif wipers_active and fog_indicators > 0:
                # Wipers on and fog indicators present - likely fog regardless of temperature
                return 'fog'

        # If temperature data is not available, fall back to the original logic
        # but improve it with additional heuristics
        # If we have wipers + snow indicators (low temp not available, but other indicators), likely snow
        if wipers_active and snow_indicators > 0 and fog_indicators == 0:
            return 'snow'
        # If we have wipers + no fog indicators, likely rain
        elif wipers_active and fog_indicators == 0:
            return 'rain'
        # If we have fog indicators but no wipers, likely fog
        elif fog_indicators > 0 and not wipers_active:
            return 'fog'
        # If we have wipers + fog indicators, check which is more likely based on context
        elif wipers_active and fog_indicators > 0:
            if CS.vEgo > 15.0 and avg_lane_prob < 0.5:  # Higher speeds with poor visibility = fog
                return 'fog'
            else:  # Lower speeds with wipers = rain/snow, decide based on additional logic
                # Without temperature, assume rain is more common than snow
                return 'rain'

    # If we have temperature data but no other complex indicators
    if ambient_temp is not None:
        if ambient_temp <= 0.0 and wipers_active:
            return 'snow'  # Definitely below freezing, likely snow
        elif ambient_temp > 4.0 and wipers_active:
            return 'rain'  # Definitely above freezing, likely rain
        elif wipers_active:
            # Between 0-4°C, could be either; use default assumption
            return 'rain'  # Rain is generally more common

    # If only wipers are active and no other indicators, default to rain (most common case)
    if wipers_active:
        return 'rain'

    # If no specific indicators of poor weather, return normal
    return 'normal'

  def _calculate_contextual_adaptive_gains(self, v_ego, thermal_state, context):
    """
    Calculate adaptive gains based on vehicle speed, thermal state and driving context.

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

    return adaptive_gains

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

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

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

        rk.monitor_time()

      # Update parameters and run extensions
      self.get_params_sp(self.sm)
      self.run_ext(self.sm, self.pm)

    finally:
      e.set()
      t.join()




def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
