"""
Adaptive Gains Controller for Controls.

This module handles adaptive gain calculations based on driving context and thermal state.
"""

import math
from typing import Any

from openpilot.common.swaglog import cloudlog


class AdaptiveGainsController:
  """Manages adaptive gain calculation and validation based on driving context and thermal state."""

  def __init__(self):
    self._prev_adaptive_gains = None

  def _validate_context_dict(self, context: dict[str, Any]) -> dict[str, Any]:
    """
    Validate the context dictionary to ensure expected keys exist and are of proper types.

    Args:
        context: Driving context information dictionary

    Returns:
        dict: A validated context dictionary with proper defaults for missing keys
    """
    validated_context: dict[str, Any] = {}

    # Validate 'is_curvy_road' key - should be boolean
    is_curvy_road_raw = context.get('is_curvy_road', False)
    validated_context['is_curvy_road'] = bool(is_curvy_road_raw) if is_curvy_road_raw is not None else False

    # Validate 'traffic_density' key - should be string
    traffic_density_raw = context.get('traffic_density', 'low')
    if not isinstance(traffic_density_raw, str):
      traffic_density_raw = str(traffic_density_raw)
    # Only allow expected values, default to 'low' if unexpected
    if traffic_density_raw not in ['low', 'medium', 'high']:
      traffic_density_raw = 'low'
    validated_context['traffic_density'] = traffic_density_raw

    # Validate 'weather_condition' key - should be string
    weather_condition_raw = context.get('weather_condition', 'normal')
    if not isinstance(weather_condition_raw, str):
      weather_condition_raw = str(weather_condition_raw)
    # Only allow expected values, default to 'normal' if unexpected
    if weather_condition_raw not in ['normal', 'rain', 'snow', 'fog', 'wind']:
      weather_condition_raw = 'normal'
    validated_context['weather_condition'] = weather_condition_raw

    # Log validation warnings for any missing or invalid keys
    if 'is_curvy_road' not in context:
      cloudlog.warning("Context key 'is_curvy_road' missing, using default: False")
    if 'traffic_density' not in context:
      cloudlog.warning("Context key 'traffic_density' missing, using default: 'low'")
    if 'weather_condition' not in context:
      cloudlog.warning("Context key 'weather_condition' missing, using default: 'normal'")

    return validated_context

  def calculate_contextual_adaptive_gains(self, v_ego: float, thermal_state: float, context: dict[str, Any]) -> dict[str, Any]:
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
      },
    }

    # Enhanced speed-dependent adjustments with more sophisticated curves
    # Use sigmoid function for smoother transition around city/highway speeds
    if v_ego < 5.0:  # Low speed - parking/stop and go
      speed_adjustment = 1.2  # Higher gains for precise low-speed control
    elif v_ego < 15.0:  # City driving ~54 km/h
      # Smooth transition using sigmoid-like function: x / (x + 1) shifted and scaled
      city_factor = 0.7 + 0.3 * (v_ego - 5.0) / (15.0 - 5.0)  # Gradually reduce from 1.0 to 0.7
      speed_adjustment = city_factor
    elif v_ego < 30.0:  # Highway driving ~54-108 km/h
      highway_factor = 0.7 - 0.2 * (v_ego - 15.0) / (30.0 - 15.0)  # Reduce from 0.7 to 0.5
      speed_adjustment = highway_factor
    else:  # Very high speed - maximum stability
      speed_adjustment = 0.4  # Minimal gains for maximum stability

    # Enhanced thermal adjustments - aggressive when very hot, conservative when cool
    if thermal_state < 0.3:
      thermal_adjustment = 1.0  # Full performance when cool
    elif thermal_state < 0.6:
      thermal_adjustment = 1.0 - (thermal_state - 0.3) * 0.5  # Gradual reduction
    else:
      thermal_adjustment = max(0.6, 1.0 - thermal_state)  # More aggressive reduction when hot

    # Validate the context dictionary to ensure required keys exist
    validated_context = self._validate_context_dict(context)

    # Enhanced context-based adjustments with more granular control
    context_adjustment = 1.0

    # Road curvature-based adjustments
    current_curvature = validated_context.get('current_curvature', 0.0)
    if current_curvature > 0.002:  # Sharp turns
      context_adjustment *= 0.7  # Very conservative for sharp turns
    elif current_curvature > 0.001:  # Moderate curves
      context_adjustment *= 0.85  # Moderate reduction for curves
    elif validated_context.get('is_curvy_road', False):  # General curvy road
      context_adjustment *= 0.9  # Slightly more conservative

    # Traffic density with more granular levels
    traffic_density = validated_context.get('traffic_density', 'low')
    if traffic_density == 'high':
      context_adjustment *= 0.75  # Very conservative in heavy traffic
    elif traffic_density == 'medium':
      context_adjustment *= 0.85  # More conservative in medium traffic
    # Low traffic remains at 1.0

    # Weather condition adjustments with different weather types
    weather_condition = validated_context.get('weather_condition', 'normal')
    if weather_condition in ['rain', 'snow']:
      context_adjustment *= 0.7  # Very conservative in precipitation
    elif weather_condition == 'fog':
      context_adjustment *= 0.75  # Conservative in low visibility
    elif weather_condition == 'wind':
      context_adjustment *= 0.85  # Moderate adjustment for wind
    # Normal weather remains at 1.0

    # Time of day adjustments (for night driving)
    time_of_day = validated_context.get('time_of_day', 'day')
    if time_of_day == 'night':
      context_adjustment *= 0.9  # Slightly more conservative at night

    # Lateral acceleration-based adjustments for stability
    lateral_accel = validated_context.get('lateral_accel', 0.0)
    if lateral_accel > 2.0:  # High lateral acceleration
      context_adjustment *= 0.8  # Reduce gains to maintain stability

    # Longitudinal acceleration magnitude
    long_accel_magnitude = validated_context.get('long_accel_magnitude', 0.0)
    if long_accel_magnitude > 2.0:  # High longitudinal acceleration
      context_adjustment *= 0.9  # Reduce gains during aggressive acceleration/braking

    # Steering activity level (high steering rate suggests challenging driving)
    steering_activity = validated_context.get('steering_activity', 0.0)
    if steering_activity > 5.0:  # High steering rate
      context_adjustment *= 0.85  # Reduce gains when steering is active

    # Apply combined adjustments with safety limits
    combined_adjustment = speed_adjustment * thermal_adjustment * context_adjustment
    combined_adjustment = max(0.3, min(1.5, combined_adjustment))  # Limit to reasonable bounds

    # Log detailed information for debugging
    cloudlog.debug(
      f"Adaptive gains calculation: v_ego={v_ego:.2f}, thermal={thermal_state:.2f}, "
      + f"speed_adj={speed_adjustment:.3f}, thermal_adj={thermal_adjustment:.3f}, "
      + f"context_adj={context_adjustment:.3f}, combined_adj={combined_adjustment:.3f}, "
      + f"curvy_road={validated_context.get('is_curvy_road', False)}, traffic={validated_context.get('traffic_density', 'low')}, "
      + f"weather={validated_context.get('weather_condition', 'normal')}, "
      + f"curvature={current_curvature:.4f}, lateral_accel={lateral_accel:.3f}, "
      + f"long_accel={long_accel_magnitude:.3f}, steering_rate={steering_activity:.2f}"
    )

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
      },
    }

    # Add validation mechanism to ensure adaptive gains are within safe bounds
    adaptive_gains = self._validate_adaptive_gains(adaptive_gains)

    # Log final adaptive gains for debugging
    cloudlog.debug(
      f"Final adaptive gains - Lateral: KP={adaptive_gains['lateral']['steer_kp']:.3f}, "
      + f"KI={adaptive_gains['lateral']['steer_ki']:.3f}, KD={adaptive_gains['lateral']['steer_kd']:.3f}; "
      + f"Longitudinal: KP={adaptive_gains['longitudinal']['accel_kp']:.3f}, "
      + f"KI={adaptive_gains['longitudinal']['accel_ki']:.3f}"
    )

    return adaptive_gains


  def _validate_adaptive_gains(self, adaptive_gains: dict[str, Any]) -> dict[str, Any]:
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
    if self._prev_adaptive_gains:
      prev_gains = self._prev_adaptive_gains

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
    cloudlog.debug(
      f"Adaptive gains validated - Lateral: KP={adaptive_gains['lateral']['steer_kp']:.3f}, "
      + f"KI={adaptive_gains['lateral']['steer_ki']:.3f}, KD={adaptive_gains['lateral']['steer_kd']:.3f}; "
      + f"Longitudinal: KP={adaptive_gains['longitudinal']['accel_kp']:.3f}, "
      + f"KI={adaptive_gains['longitudinal']['accel_ki']:.3f}"
    )

    return adaptive_gains

