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
    Calculate optimized adaptive gains based on vehicle speed, thermal state and driving context.
    Reduced computational overhead while maintaining safety.

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
    speed_factor = min(1.0, v_ego / 30.0)
    # Reduce gains by up to 30% at high speeds for enhanced stability
    speed_adjustment = 1.0 - (0.3 * speed_factor)

    # Thermal adjustments - reduced impact to maintain performance
    thermal_adjustment = 1.0 - (thermal_state * 0.15)  # Slightly less reduction

    # Context-based adjustments - simplified
    context_adjustment = 1.0

    # Reduce gains on curvy roads for smoother steering
    if context.get('is_curvy_road', False):
      context_adjustment *= 0.85

    # Reduce gains in high traffic or poor weather
    if context.get('traffic_density', 'low') == 'high' or context.get('weather_condition', 'normal') != 'normal':
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
      },
    }

    # Apply lightweight validation
    adaptive_gains = self._validate_adaptive_gains_optimized(adaptive_gains)

    return adaptive_gains


  def _validate_adaptive_gains_optimized(self, adaptive_gains: dict[str, Any]) -> dict[str, Any]:
    """
    Lightweight validation of adaptive gains with reduced computational overhead.

    Args:
        adaptive_gains: Dictionary containing lateral and longitudinal gains

    Returns:
        dict: Validated and potentially corrected adaptive gain parameters
    """
    # Define safe bounds for gains
    MIN_STEER_KP = 0.1
    MAX_STEER_KP = 3.0
    MIN_STEER_KI = 0.01
    MAX_STEER_KI = 1.0
    MIN_STEER_KD = 0.0
    MAX_STEER_KD = 0.1
    MIN_ACCEL_KP = 0.1
    MAX_ACCEL_KP = 2.0
    MIN_ACCEL_KI = 0.01
    MAX_ACCEL_KI = 1.0

    # Simple validation without complex change detection
    if 'lateral' in adaptive_gains:
      lateral = adaptive_gains['lateral']
      if 'steer_kp' in lateral:
        lateral['steer_kp'] = max(MIN_STEER_KP, min(MAX_STEER_KP, lateral['steer_kp']))
      if 'steer_ki' in lateral:
        lateral['steer_ki'] = max(MIN_STEER_KI, min(MAX_STEER_KI, lateral['steer_ki']))
      if 'steer_kd' in lateral:
        lateral['steer_kd'] = max(MIN_STEER_KD, min(MAX_STEER_KD, lateral['steer_kd']))

    if 'longitudinal' in adaptive_gains:
      longitudinal = adaptive_gains['longitudinal']
      if 'accel_kp' in longitudinal:
        longitudinal['accel_kp'] = max(MIN_ACCEL_KP, min(MAX_ACCEL_KP, longitudinal['accel_kp']))
      if 'accel_ki' in longitudinal:
        longitudinal['accel_ki'] = max(MIN_ACCEL_KI, min(MAX_ACCEL_KI, longitudinal['accel_ki']))

    # Basic NaN/Infinity checks
    for gain_type in adaptive_gains:
      for gain_name, gain_value in adaptive_gains[gain_type].items():
        if not math.isfinite(gain_value):
          # Set to safe default
          if 'steer' in gain_name:
            adaptive_gains[gain_type][gain_name] = 1.0
          elif 'accel' in gain_name:
            adaptive_gains[gain_type][gain_name] = 1.0

    # Store for next iteration (minimal processing)
    self._prev_adaptive_gains = adaptive_gains.copy()

    return adaptive_gains

  def _validate_adaptive_gains(self, adaptive_gains: dict[str, Any]) -> dict[str, Any]:
    """
    Legacy validation function kept for compatibility (calls the optimized version).
    """
    return self._validate_adaptive_gains_optimized(adaptive_gains)
