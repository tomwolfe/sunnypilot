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
    cloudlog.debug(
      f"Adaptive gains calculation: v_ego={v_ego:.2f}, thermal={thermal_state:.2f}, "
      + f"speed_factor={speed_factor:.3f}, thermal_adj={thermal_adjustment:.3f}, "
      + f"context_adj={context_adjustment:.3f}, combined_adj={combined_adjustment:.3f}, "
      + f"curvy_road={context['is_curvy_road']}, traffic={context['traffic_density']}, "
      + f"weather={context['weather_condition']}"
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
