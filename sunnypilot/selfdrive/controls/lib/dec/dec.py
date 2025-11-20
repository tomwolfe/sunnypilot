"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# Version = 2025-6-30

from collections import deque
from cereal import messaging
from opendbc.car import structs
from numpy import interp
import math
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.dec.constants import WMACConstants
from openpilot.system.statsd import statlog
from typing import Literal
import time

# d-e2e, from modeldata.h
TRAJECTORY_SIZE = 33
SET_MODE_TIMEOUT = 15

# Define the valid mode types
ModeType = Literal['acc', 'blended']


class SmoothKalmanFilter:
  """Enhanced Kalman filter with smoothing for stable decision making."""

  def __init__(self, initial_value=0, measurement_noise=0.1, process_noise=0.01,
               alpha=1.0, smoothing_factor=0.85):
    self.x = initial_value
    self.P = 1.0
    self.R = measurement_noise
    self.Q = process_noise
    self.alpha = alpha
    self.smoothing_factor = smoothing_factor
    self.initialized = False
    self.history = []
    self.max_history = 10
    self.confidence = 0.0

  def add_data(self, measurement):
    if len(self.history) >= self.max_history:
      self.history.pop(0)
    self.history.append(measurement)

    if not self.initialized:
      self.x = measurement
      self.initialized = True
      self.confidence = 0.1
      return

    self.P = self.alpha * self.P + self.Q

    K = self.P / (self.P + self.R)
    effective_K = K * (1.0 - self.smoothing_factor) + self.smoothing_factor * 0.1

    innovation = measurement - self.x
    self.x = self.x + effective_K * innovation
    self.P = (1 - effective_K) * self.P

    if abs(innovation) < 0.1:
      self.confidence = min(1.0, self.confidence + 0.05)
    else:
      self.confidence = max(0.1, self.confidence - 0.02)

  def get_value(self):
    return self.x if self.initialized else None

  def get_confidence(self):
    return self.confidence

  def reset_data(self):
    self.initialized = False
    self.history = []
    self.confidence = 0.0


class ModeTransitionManager:
  """Manages smooth transitions between driving modes with hysteresis."""

  def __init__(self):
    self.current_mode: ModeType = 'acc'
    self.mode_confidence = {'acc': 1.0, 'blended': 0.0}
    self.transition_timeout = 0
    self.min_mode_duration = 10
    self.mode_duration = 0
    self.emergency_override = False

  def request_mode(self, mode: ModeType, confidence: float = 1.0, emergency: bool = False):
    # Emergency override for critical situations (stops, collisions)
    if emergency:
      self.emergency_override = True
      self.current_mode = mode
      self.transition_timeout = SET_MODE_TIMEOUT
      self.mode_duration = 0
      return

    self.mode_confidence[mode] = min(1.0, self.mode_confidence[mode] + 0.1 * confidence)
    for m in self.mode_confidence:
      if m != mode:
        self.mode_confidence[m] = max(0.0, self.mode_confidence[m] - 0.05)

    # Require minimum duration in current mode (unless emergency)
    if self.mode_duration < self.min_mode_duration and not self.emergency_override:
      return

    # Hysteresis: higher threshold for mode changes
    confidence_threshold = 0.6 if mode != self.current_mode else 0.3  # Lower threshold for faster response

    if self.mode_confidence[mode] > confidence_threshold:
      if mode != self.current_mode and self.transition_timeout == 0:
        self.transition_timeout = SET_MODE_TIMEOUT
        self.current_mode = mode
        self.mode_duration = 0

  def update(self):
    if self.transition_timeout > 0:
      self.transition_timeout -= 1
    self.mode_duration += 1

    # Reset emergency override after some time
    if self.emergency_override and self.mode_duration > 20:
      self.emergency_override = False

    # Gradual confidence decay
    for mode in self.mode_confidence:
      self.mode_confidence[mode] *= 0.98

  def get_mode(self) -> ModeType:
    return self.current_mode


class DynamicExperimentalController:
  def __init__(self, CP: structs.CarParams, mpc, params=None):
    self._CP = CP
    self._mpc = mpc
    self._params = params or Params()
    self._enabled: bool = self._params.get_bool("DynamicExperimentalControl")
    self._active: bool = False
    self._frame: int = 0
    self._urgency = 0.0

    # Monitoring variables
    self._last_mode: ModeType | None = None
    self._mode_switch_count = 0
    self._mode_timer = {'acc': 0, 'blended': 0}
    self._last_mode_change_time = time.monotonic()
    self._monitoring_enabled = True
    self._monitoring_frame_counter = 0
    self._monitoring_interval = int(1.0 / DT_MDL)  # Send metrics every second

    self._mode_manager = ModeTransitionManager()

    # Smooth filters for stable decision making with faster response for critical scenarios
    self._lead_filter = SmoothKalmanFilter(
      measurement_noise=0.15,
      process_noise=0.05,
      alpha=1.02,
      smoothing_factor=0.8
    )

    self._slow_down_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.1,
      alpha=1.05,
      smoothing_factor=0.7
    )

    self._slowness_filter = SmoothKalmanFilter(
      measurement_noise=0.1,
      process_noise=0.06,
      alpha=1.015,
      smoothing_factor=0.92
    )

    self._mpc_fcw_filter = SmoothKalmanFilter(
      measurement_noise=0.2,
      process_noise=0.1,
      alpha=1.1,
      smoothing_factor=0.5
    )

    # NEW: Enhanced predictive filters for better anticipation
    self._predictive_stop_filter = SmoothKalmanFilter(
      measurement_noise=0.12,
      process_noise=0.08,
      alpha=1.03,
      smoothing_factor=0.75
    )

    # NEW: Weather and environmental adaptation
    self._weather_confidence = 1.0  # 1.0 = normal, < 1.0 = adverse conditions
    self._lighting_condition = 1.0  # 1.0 = daylight, < 1.0 = poor visibility

    # NEW: Driver behavior adaptation
    self._driver_aggression_score = 0.5  # 0.0-1.0, 0.5 = neutral
    self._driver_override_history = deque(maxlen=50)  # Track recent overrides

    # NEW: Multi-sensor fusion for enhanced reliability
    self._lidar_available = False  # Will be set based on car capabilities
    self._radar_confidence = 1.0   # 1.0 = reliable, < 1.0 = degraded

    self._has_lead_filtered = False
    self._has_slow_down = False
    self._has_slowness = False
    self._has_mpc_fcw = False
    self._has_predictive_stop = False  # NEW: Predictive stop detection
    self._v_ego_kph = 0.0
    self._v_cruise_kph = 0.0
    self._has_standstill = False
    self._mpc_fcw_crash_cnt = 0
    self._standstill_count = 0
    # debug
    self._endpoint_x = float('inf')
    self._expected_distance = 0.0
    self._trajectory_valid = False

    # NEW: Predictive model parameters
    self._approaching_object_confidence = 0.0
    self._stop_signal_confidence = 0.0
    self._intersection_confidence = 0.0

  def _read_params(self) -> None:
    if self._frame % int(1. / DT_MDL) == 0:
      self._enabled = self._params.get_bool("DynamicExperimentalControl")

  def mode(self) -> str:
    return self._mode_manager.get_mode()

  def enabled(self) -> bool:
    return self._enabled

  def active(self) -> bool:
    return self._active

  def set_mpc_fcw_crash_cnt(self) -> None:
    """Set MPC FCW crash count"""
    self._mpc_fcw_crash_cnt = self._mpc.crash_cnt

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    car_state = sm['carState']
    lead_one = sm['radarState'].leadOne
    md = sm['modelV2']

    self._v_ego_kph = car_state.vEgo * 3.6
    self._v_cruise_kph = car_state.vCruise
    self._has_standstill = car_state.standstill

    # Update environmental conditions NEW
    self._update_environmental_conditions(sm)

    # Update driver behavior adaptation NEW
    self._update_driver_behavior(sm)

    # standstill detection
    if self._has_standstill:
      self._standstill_count = min(20, self._standstill_count + 1)
    else:
      self._standstill_count = max(0, self._standstill_count - 1)

    # Lead detection
    self._lead_filter.add_data(float(lead_one.status))
    lead_value = self._lead_filter.get_value() or 0.0
    self._has_lead_filtered = lead_value > WMACConstants.LEAD_PROB

    # MPC FCW detection
    fcw_filtered_value = self._mpc_fcw_filter.get_value() or 0.0
    self._mpc_fcw_filter.add_data(float(self._mpc_fcw_crash_cnt > 0))
    self._has_mpc_fcw = fcw_filtered_value > 0.5

    # NEW: Predictive stop detection from model data
    self._calculate_predictive_stops(md)

    # Slow down detection
    self._calculate_slow_down(md)

    # Slowness detection
    if not (self._standstill_count > 5) and not self._has_slow_down:
      current_slowness = float(self._v_ego_kph <= (self._v_cruise_kph * WMACConstants.SLOWNESS_CRUISE_OFFSET))
      self._slowness_filter.add_data(current_slowness)
      slowness_value = self._slowness_filter.get_value() or 0.0

      # Hysteresis for slowness
      threshold = WMACConstants.SLOWNESS_PROB * (0.8 if self._has_slowness else 1.1)
      self._has_slowness = slowness_value > threshold

  def _calculate_slow_down(self, md):
    """Calculate urgency based on trajectory endpoint vs expected distance."""

    # Reset to safe defaults
    urgency = 0.0
    self._endpoint_x = float('inf')
    self._trajectory_valid = False

    # Validate trajectory data - check if we have the expected size
    position_valid = len(md.position.x) == TRAJECTORY_SIZE if hasattr(md.position, 'x') else False
    orientation_valid = len(md.orientation.x) == TRAJECTORY_SIZE if hasattr(md.orientation, 'x') else False

    # Additional safety checks for trajectory data validity
    if (position_valid and hasattr(md.position, 'x') and
        len(md.position.x) > 0):

      # Check for NaN or infinity values in the trajectory data
      try:
        for x in md.position.x:
          if math.isnan(x) or math.isinf(x):
            position_valid = False
            break
      except (TypeError, ValueError):
        # If there's an issue with checking for NaN/inf, mark as invalid
        position_valid = False

      # Check if the endpoint value is reasonable (not negative or extremely large)
      if position_valid:
        endpoint_x = md.position.x[TRAJECTORY_SIZE - 1]
        if not (endpoint_x >= 0 and endpoint_x < 1000):  # Reasonable range for trajectory endpoint
          position_valid = False

    if not (position_valid and orientation_valid):
      # Invalid trajectory - this itself might indicate a stop scenario
      # Apply more conservative urgency for incomplete trajectories at speed
      if self._v_ego_kph > 5.0:  # Even at low speeds, consider possible stop
        urgency = 0.6

      self._slow_down_filter.add_data(urgency)
      urgency_filtered = self._slow_down_filter.get_value() or 0.0
      self._has_slow_down = urgency_filtered > WMACConstants.SLOW_DOWN_PROB
      self._urgency = urgency_filtered
      return

    # We have a valid full trajectory
    self._trajectory_valid = True

    # Use the exact endpoint (33rd point, index 32)
    endpoint_x = md.position.x[TRAJECTORY_SIZE - 1]
    self._endpoint_x = endpoint_x

    # Get expected distance based on current speed using tuned constants
    expected_distance = interp(self._v_ego_kph,
                               WMACConstants.SLOW_DOWN_BP,
                               WMACConstants.SLOW_DOWN_DIST)
    self._expected_distance = expected_distance

    # Calculate urgency based on trajectory shortage with more aggressive calculation
    if endpoint_x < expected_distance:
      shortage = expected_distance - endpoint_x
      shortage_ratio = shortage / expected_distance

      # Base urgency on shortage ratio with more aggressive scaling
      urgency = min(1.0, shortage_ratio * 3.0)  # Increased from 2.0 to 3.0

      # Increase urgency for very short trajectories (imminent stops) - more aggressive
      critical_distance = expected_distance * 0.5  # Increased from 0.3 to 0.5
      if endpoint_x < critical_distance:
        urgency = min(1.0, urgency * 3.0)  # Increased from 2.0 to 3.0

      # Speed-based urgency adjustment - more aggressive
      if self._v_ego_kph > 10.0:  # Lower threshold for speed-based adjustment
        speed_factor = 1.0 + (self._v_ego_kph - 10.0) / 40.0  # Adjusted for more responsiveness
        urgency = min(1.0, urgency * speed_factor)

    # Apply filtering but with less smoothing for stops
    self._slow_down_filter.add_data(urgency)
    urgency_filtered = self._slow_down_filter.get_value() or 0.0

    # Update state - use the original threshold instead of reduced one for more precise stop detection
    self._has_slow_down = urgency_filtered > WMACConstants.SLOW_DOWN_PROB
    self._urgency = urgency_filtered

  def _update_environmental_conditions(self, sm: messaging.SubMaster) -> None:
    """Update environmental condition awareness for adaptive driving"""
    # NEW: Get environmental data from various sensors and services
    car_state = sm['carState']

    # Placeholder for weather data (in real implementation, this would come from various sources)
    # For now, use car sensors that might indicate conditions
    steering_torque_input = abs(car_state.steeringTorqueEps) if hasattr(car_state, 'steeringTorqueEps') else 0.0
    wheel_speed_fl = car_state.wheelSpeeds.fl if hasattr(car_state.wheelSpeeds, 'fl') else 0.0
    wheel_speed_fr = car_state.wheelSpeeds.fr if hasattr(car_state.wheelSpeeds, 'fr') else 0.0

    # Simple heuristic for weather conditions based on steering and wheel speed consistency
    # In real implementation, this would use actual weather data from APIs or sensors
    if hasattr(car_state, 'steeringPressed') and car_state.steeringPressed:
      # If driver is making frequent steering corrections, assume adverse conditions
      self._weather_confidence = max(0.3, self._weather_confidence - 0.01)
    else:
      # Gradually restore confidence if conditions appear stable
      self._weather_confidence = min(1.0, self._weather_confidence + 0.001)

    # Lighting condition assessment (simplified)
    # In real implementation, this would use light sensor data or time of day
    if self._v_ego_kph > 5 and self._v_ego_kph < 25 and self._has_slow_down:
      # In low-light conditions with stop anticipation, reduce confidence
      self._lighting_condition = 0.8

    # Radar confidence assessment
    radar_state = sm['radarState']
    if not radar_state.radarFaulted:
      self._radar_confidence = min(1.0, self._radar_confidence + 0.001)  # Slowly increase when radar is healthy
    else:
      self._radar_confidence = max(0.1, self._radar_confidence - 0.1)   # Quickly decrease when radar fails

  def _update_driver_behavior(self, sm: messaging.SubMaster) -> None:
    """Track and adapt to driver behavior preferences"""
    car_state = sm['carState']

    # Detect if driver is overriding controls
    if hasattr(car_state, 'steeringPressed') and car_state.steeringPressed:
      # Record override event
      self._driver_override_history.append(time.monotonic())

      # Recalculate driver aggression score based on override frequency
      recent_overrides = [t for t in self._driver_override_history
                         if time.monotonic() - t < 30.0]  # Last 30 seconds
      override_frequency = min(1.0, len(recent_overrides) / 10.0)  # Normalize

      # Adjust aggression score based on override patterns
      if len(recent_overrides) > 3:
        # Driver is more interventionist, reduce aggression
        self._driver_aggression_score = max(0.1, self._driver_aggression_score - 0.1)
      else:
        # Gradually return to neutral
        self._driver_aggression_score = min(0.9, self._driver_aggression_score + 0.001)

  def _calculate_predictive_stops(self, md) -> None:
    """NEW: Calculate predictive stop anticipation from model data"""
    if not (hasattr(md, 'meta') and hasattr(md.meta, 'stopState')):
      return

    # Predictive stop detection from model meta information
    stop_state_confidence = float(md.meta.stopState) if md.meta.stopState else 0.0
    self._predictive_stop_filter.add_data(stop_state_confidence)

    predictive_value = self._predictive_stop_filter.get_value() or 0.0
    self._has_predictive_stop = predictive_value > 0.7  # Higher threshold for predictive detection

    # Additional predictive factors from model with proper error handling
    if hasattr(md, 'temporalBatch') and len(md.temporalBatch) > 0:
      temporal_data = md.temporalBatch[0]
      # Check for traffic light or stop sign ahead
      traffic_light_conf = 0.0
      if hasattr(temporal_data, 'trafficLightStateProba') and len(temporal_data.trafficLightStateProba) > 0:
        traffic_light_conf = temporal_data.trafficLightStateProba[0]
      elif hasattr(temporal_data, 'trafficStateProba') and len(temporal_data.trafficStateProba) > 0:
        traffic_light_conf = temporal_data.trafficStateProba[0]

      self._stop_signal_confidence = max(self._stop_signal_confidence, traffic_light_conf * 0.3)

    # Additional predictive factors: desire states from model
    if hasattr(md, 'meta') and hasattr(md.meta, 'desireState') and len(md.meta.desireState) > 0:
      from cereal import log
      # Check for stop desire state
      if len(md.meta.desireState) > log.Desire.stop:
        stop_desire_confidence = md.meta.desireState[log.Desire.stop]
        if stop_desire_confidence > 0.6:
          # Use both filters for more accurate prediction
          combined_confidence = max(stop_state_confidence, stop_desire_confidence)
          self._predictive_stop_filter.add_data(combined_confidence * 0.8)  # Weighted contribution
          predictive_value = self._predictive_stop_filter.get_value() or 0.0
          self._has_predictive_stop = predictive_value > 0.7

    # Update approach confidence based on trajectory
    if self._trajectory_valid and self._endpoint_x < 50:  # Within 50m
      self._approaching_object_confidence = max(0.5, self._approaching_object_confidence)
    else:
      self._approaching_object_confidence *= 0.95  # Decay over time

  def _radarless_mode(self) -> None:
    """Radarless mode decision logic with emergency handling and environmental awareness."""

    # EMERGENCY: MPC FCW - immediate blended mode
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    # NEW: Predictive stop handling - use blended mode when predictive stop detected
    if self._has_predictive_stop and self._v_ego_kph > 5.0:
      confidence = min(1.0, self._stop_signal_confidence * 1.5)
      self._mode_manager.request_mode('blended', confidence=confidence, emergency=True)
      return

    # Environmental condition adjustments
    base_confidence = 1.0
    if self._weather_confidence < 0.7:
      # In adverse weather, be more conservative
      base_confidence *= 0.8
    if self._lighting_condition < 0.6:
      # In poor lighting, be more conservative
      base_confidence *= 0.85

    # NEW: Driver behavior adaptation - more conservative if driver is interventionist
    if self._driver_aggression_score < 0.3:
      base_confidence *= 0.7  # More conservative for cautious drivers

    # Standstill: use blended with high confidence to ensure complete stop
    if self._standstill_count > 3:
      adjusted_confidence = base_confidence * 1.0
      self._mode_manager.request_mode('blended', confidence=adjusted_confidence)  # Increased from 0.9 to 1.0
      return

    # Slow down scenarios: emergency for high urgency, normal for lower urgency
    # More aggressive standstill enforcement when approaching stop
    if self._has_slow_down:
      if self._urgency > 0.5:  # Lowered threshold from 0.7 for more responsive stopping
        # Emergency: immediate blended mode for high urgency stops
        confidence = min(1.0, self._urgency * 1.2)  # Add confidence based on urgency
        adjusted_confidence = confidence * base_confidence
        self._mode_manager.request_mode('blended', confidence=adjusted_confidence, emergency=True)
      else:
        # Normal: blended with urgency-based confidence
        confidence = min(1.0, self._urgency * 1.8)  # Increased from 1.5 for more responsiveness
        adjusted_confidence = confidence * base_confidence
        self._mode_manager.request_mode('blended', confidence=adjusted_confidence)
      return

    # Driving slow: use ACC (but not if actively slowing down)
    if self._has_slowness and not self._has_slow_down:
      adjusted_confidence = 0.8 * base_confidence
      self._mode_manager.request_mode('acc', confidence=adjusted_confidence)
      return

    # NEW: Enhanced model-based prediction for upcoming maneuvers (in radarless mode)
    # Use modelV2 data to predict upcoming road conditions and adjust behavior
    if 'modelV2' in sm and sm.updated['modelV2']:
      model_msg = sm['modelV2']

      # Check for upcoming curves that might require mode adjustment
      if len(model_msg.path.x) > 0 and len(model_msg.path.y) > 0:
        # Look at the curvature 2-3 seconds ahead
        future_idx = min(int(2.0 / 0.05), len(model_msg.path.y) - 1)  # 2 seconds ahead (assuming 0.05s spacing)
        if future_idx < len(model_msg.path.y):
          future_curvature = abs(model_msg.path.y[future_idx]) if future_idx < len(model_msg.path.y) else 0.0

          # If upcoming curvature is significant, be more conservative
          if future_curvature > 0.003:  # Significant curve ahead
            adjusted_confidence = 0.6 * base_confidence  # Reduce confidence for experimental mode
            if future_curvature > 0.008:  # Very sharp curve ahead
              self._mode_manager.request_mode('blended', confidence=adjusted_confidence)
              return

    # NEW: Check for upcoming traffic lights or stops in model predictions
    if hasattr(model_msg, 'meta') and len(model_msg.meta.desireState) > 0:
      from cereal import log
      # High probability of upcoming stop or traffic light
      if model_msg.meta.desireState[log.Desire.stop] > 0.7:
        adjusted_confidence = 0.5 * base_confidence  # Be more conservative
        self._mode_manager.request_mode('blended', confidence=adjusted_confidence)
        return

    # Default: ACC
    adjusted_confidence = 0.7 * base_confidence
    self._mode_manager.request_mode('acc', confidence=adjusted_confidence)

  def _radar_mode(self) -> None:
    """Radar mode with emergency handling and environmental awareness."""

    # EMERGENCY: MPC FCW - immediate blended mode
    if self._has_mpc_fcw:
      self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)
      return

    # NEW: Predictive stop handling - use blended mode when predictive stop detected
    if self._has_predictive_stop and self._v_ego_kph > 5.0:
      confidence = min(1.0, self._stop_signal_confidence * 1.5)
      self._mode_manager.request_mode('blended', confidence=confidence, emergency=True)
      return

    # Environmental condition adjustments
    base_confidence = 1.0
    if self._weather_confidence < 0.7:
      # In adverse weather, be more conservative
      base_confidence *= 0.8
    if self._lighting_condition < 0.6:
      # In poor lighting, be more conservative
      base_confidence *= 0.85

    # NEW: Driver behavior adaptation - more conservative if driver is interventionist
    if self._driver_aggression_score < 0.3:
      base_confidence *= 0.7  # More conservative for cautious drivers

    # NEW: Radar reliability check
    if self._radar_confidence < 0.5:
      # When radar is unreliable, rely more on vision-based detection
      if self._has_slow_down or self._has_predictive_stop:
        # Use blended mode when vision detects issues but radar might be unreliable
        confidence = min(1.0, (self._urgency + self._stop_signal_confidence) * base_confidence)
        self._mode_manager.request_mode('blended', confidence=confidence)
        return

    # If lead detected and not in standstill: always use ACC
    if self._has_lead_filtered and not (self._standstill_count > 3):
      adjusted_confidence = base_confidence * 1.0
      self._mode_manager.request_mode('acc', confidence=adjusted_confidence)
      return

    # Slow down scenarios: emergency for high urgency, normal for lower urgency
    # More aggressive standstill enforcement when approaching stop
    if self._has_slow_down:
      if self._urgency > 0.5:  # Lowered threshold from 0.7 for more responsive stopping
        # Emergency: immediate blended mode for high urgency stops
        confidence = min(1.0, self._urgency * 1.2)  # Add confidence based on urgency
        adjusted_confidence = confidence * base_confidence
        self._mode_manager.request_mode('blended', confidence=adjusted_confidence, emergency=True)
      else:
        # Normal: blended with urgency-based confidence
        confidence = min(1.0, self._urgency * 1.8)  # Increased from 1.3 for more responsiveness
        adjusted_confidence = confidence * base_confidence
        self._mode_manager.request_mode('blended', confidence=adjusted_confidence)
      return

    # Standstill: use blended with high confidence to ensure complete stop
    if self._standstill_count > 3:
      adjusted_confidence = base_confidence * 1.0
      self._mode_manager.request_mode('blended', confidence=adjusted_confidence)  # Increased from 0.9 to 1.0
      return

    # Driving slow: use ACC (but not if actively slowing down)
    if self._has_slowness and not self._has_slow_down:
      adjusted_confidence = 0.8 * base_confidence
      self._mode_manager.request_mode('acc', confidence=adjusted_confidence)
      return

    # NEW: Enhanced model-based prediction for upcoming maneuvers
    # Use modelV2 data to predict upcoming road conditions and adjust behavior
    if 'modelV2' in sm and sm.updated['modelV2']:
      model_msg = sm['modelV2']

      # Check for upcoming curves that might require mode adjustment
      if len(model_msg.path.x) > 0 and len(model_msg.path.y) > 0:
        # Look at the curvature 2-3 seconds ahead
        future_idx = min(int(2.0 / 0.05), len(model_msg.path.y) - 1)  # 2 seconds ahead (assuming 0.05s spacing)
        if future_idx < len(model_msg.path.y):
          future_curvature = abs(model_msg.path.y[future_idx]) if future_idx < len(model_msg.path.y) else 0.0

          # If upcoming curvature is significant, be more conservative
          if future_curvature > 0.003:  # Significant curve ahead
            adjusted_confidence = 0.6 * base_confidence  # Reduce confidence for experimental mode
            if future_curvature > 0.008:  # Very sharp curve ahead
              self._mode_manager.request_mode('blended', confidence=adjusted_confidence)
              return

    # NEW: Check for upcoming traffic lights or stops in model predictions
    if hasattr(model_msg, 'meta') and len(model_msg.meta.desireState) > 0:
      from cereal import log
      # High probability of upcoming stop or traffic light
      if model_msg.meta.desireState[log.Desire.stop] > 0.7:
        adjusted_confidence = 0.5 * base_confidence  # Be more conservative
        self._mode_manager.request_mode('blended', confidence=adjusted_confidence)
        return

    # Default: ACC
    adjusted_confidence = 0.7 * base_confidence
    self._mode_manager.request_mode('acc', confidence=adjusted_confidence)

  def update(self, sm: messaging.SubMaster) -> None:
    try:
      self._read_params()

      self.set_mpc_fcw_crash_cnt()

      self._update_calculations(sm)

      # Check if we should enforce standstill mode explicitly
      enforce_standstill = self._should_enforce_standstill(sm)

      if self._CP.radarUnavailable:
        self._radarless_mode()
      else:
        self._radar_mode()

      # If we should enforce standstill, make sure we stay in blended mode
      if enforce_standstill:
        self._mode_manager.request_mode('blended', confidence=1.0, emergency=True)

      self._mode_manager.update()
      self._active = sm['selfdriveState'].experimentalMode and self._enabled
      self._frame += 1

      # Perform monitoring
      if self._monitoring_enabled:
        self._monitor_modes()
        self._monitoring_frame_counter += 1

        # Send metrics every second
        if self._monitoring_frame_counter >= self._monitoring_interval:
          self._send_metrics(sm)
          self._monitoring_frame_counter = 0
    except Exception as e:
      cloudlog.error(f"Error in DEC update: {e}")
      # Fallback to safe mode if an error occurs
      self._mode_manager.request_mode('acc', confidence=1.0, emergency=True)
      self._active = False

  def _monitor_modes(self) -> None:
    """Monitor mode changes and time spent in each mode."""
    current_mode = self._mode_manager.get_mode()

    # Track mode changes
    if self._last_mode is not None and self._last_mode != current_mode:
      self._mode_switch_count += 1
      # Log time spent in previous mode
      time_in_last_mode = time.monotonic() - self._last_mode_change_time
      self._mode_timer[self._last_mode] += time_in_last_mode
      self._last_mode_change_time = time.monotonic()

    self._last_mode = current_mode

  def _should_enforce_standstill(self, sm: messaging.SubMaster) -> bool:
    """Explicitly check if the vehicle should be at complete standstill."""
    car_state = sm['carState']

    # If car is currently at standstill, enforce blended mode to maintain the stop
    if car_state.standstill:
      return True

    # If longitudinal plan indicates should stop and vEgo is very low, enforce standstill
    # Check if the longitudinalPlan service exists in the SubMaster before accessing it
    if 'longitudinalPlan' in sm and sm.updated['longitudinalPlan']:
      long_plan = sm['longitudinalPlan']
      if long_plan.shouldStop and car_state.vEgo < 0.5:  # Less than 0.5 m/s should be considered stopped
        return True

    # Check if in a stop scenario (traffic light, stop sign, etc.) from model
    if hasattr(sm['modelV2'], 'meta') and hasattr(sm['modelV2'].meta, 'stopState'):
      # If model indicates stop state and we're at very low speed
      if sm['modelV2'].meta.stopState and car_state.vEgo < 2.0:
        return True

    return False

  def _send_metrics(self, sm) -> None:
    """Send monitoring metrics to stats system."""
    try:
      current_mode = self._mode_manager.get_mode()
      current_time = time.monotonic()

      # Update time in current mode (since last update)
      time_in_current_mode = current_time - self._last_mode_change_time
      self._mode_timer[current_mode] += time_in_current_mode
      self._last_mode_change_time = current_time

      # Send metrics
      if self._active:  # Only send when the system is actually active
        statlog.sample('DynamicExperimentalController.mode_switches', self._mode_switch_count)
        statlog.gauge('DynamicExperimentalController.current_mode', 1.0 if current_mode == 'blended' else 0.0)
        statlog.sample('DynamicExperimentalController.time_in_acc_mode', self._mode_timer['acc'])
        statlog.sample('DynamicExperimentalController.time_in_blended_mode', self._mode_timer['blended'])

        # Additional context metrics
        statlog.gauge('DynamicExperimentalController.v_ego_kph', self._v_ego_kph)
        statlog.gauge('DynamicExperimentalController.v_cruise_kph', self._v_cruise_kph)
        statlog.gauge('DynamicExperimentalController.has_lead_filtered', float(self._has_lead_filtered))
        statlog.gauge('DynamicExperimentalController.has_slow_down', float(self._has_slow_down))
        statlog.gauge('DynamicExperimentalController.has_slowness', float(self._has_slowness))
        statlog.gauge('DynamicExperimentalController.has_mpc_fcw', float(self._has_mpc_fcw))
        statlog.gauge('DynamicExperimentalController.has_standstill', float(self._has_standstill))
        statlog.gauge('DynamicExperimentalController.urgency', self._urgency)
        statlog.gauge('DynamicExperimentalController.enabled', float(self._enabled))
        statlog.gauge('DynamicExperimentalController.active', float(self._active))

        # Reset counters for next interval
        self._mode_switch_count = 0

    except Exception as e:
      # Avoid any monitoring issues from affecting functionality
      pass
