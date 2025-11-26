"""
Advanced safety features and fail-safes for autonomous driving.

This module provides enhanced safety mechanisms to improve the reliability
and safety of the autonomous driving system by implementing various checks
and fail-safe procedures.
"""

from collections import deque
import numpy as np
from cereal import car, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from opendbc.car.interfaces import LatCtrlState, StateTrans


class SafetyManager:
  """
  Advanced safety manager that monitors various system parameters and
  provides fail-safe mechanisms when safety limits are exceeded.
  """
  
  def __init__(self):
    self.params = Params()
    
    # System state monitoring
    self.steering_torque_history = deque(maxlen=20)  # 0.2 seconds at 100Hz
    self.acceleration_history = deque(maxlen=20)      # 0.2 seconds at 100Hz
    self.steering_angle_history = deque(maxlen=10)    # 0.1 seconds at 100Hz
    
    # Safety thresholds (configurable)
    self.max_unexpected_torque = self.params.get_float("SafetyMaxUnexpectedTorque", 200.0)
    self.max_acceleration_change = self.params.get_float("SafetyMaxAccelerationChange", 5.0)  # m/s^3
    self.max_steering_rate = self.params.get_float("SafetyMaxSteeringRate", 100.0)  # deg/s
    self.min_model_prediction_confidence = self.params.get_float("SafetyMinModelConfidence", 0.5)
    
    # Safety state
    self.safety_violation_count = 0
    self.max_safety_violations = 3  # Maximum violations before disengagement
    self.safety_engaged = True  # Whether safety system is actively monitoring
    self.violation_threshold = 0.5  # Threshold for triggering safety actions
    
    # Initialize state variables
    self._last_steering_angle = 0.0
    self._last_acceleration = 0.0
    self._last_update_time = 0.0
    self._initialized = False
    
  def update_params(self):
    """Update safety parameters from system parameters"""
    try:
      self.max_unexpected_torque = float(self.params.get("SafetyMaxUnexpectedTorque", "200.0"))
      self.max_acceleration_change = float(self.params.get("SafetyMaxAccelerationChange", "5.0"))
      self.max_steering_rate = float(self.params.get("SafetyMaxSteeringRate", "100.0"))
      self.min_model_prediction_confidence = float(self.params.get("SafetyMinModelConfidence", "0.5"))
    except (TypeError, ValueError):
      # Use defaults if parameters are invalid
      pass

  def monitor_steering_safety(self, car_state: car.CarState, control_output: car.CarControl.ActuatorsOutput) -> bool:
    """
    Monitor steering safety by checking for unexpected torques or rapid changes.
    
    Args:
      car_state: Current car state
      control_output: Control system output
      
    Returns:
      True if steering is safe, False otherwise
    """
    # Check for excessive steering torque
    if abs(car_state.steeringTorque) > self.max_unexpected_torque:
      # Check if the high torque is expected based on our control request
      expected_torque = abs(control_output.torque) if control_output else 0
      if abs(car_state.steeringTorque) > expected_torque * 1.5:  # 150% of expected
        return False
    
    # Check for rapid steering rate changes
    current_angle = car_state.steeringAngleDeg
    if self._initialized:
      time_diff = DT_CTRL  # Fixed time step
      if time_diff > 0:
        steering_rate = abs(current_angle - self._last_steering_angle) / time_diff
        if steering_rate > self.max_steering_rate:
          return False
    
    # Store current values for next iteration
    self.steering_torque_history.append(abs(car_state.steeringTorque))
    self.steering_angle_history.append(current_angle)
    self._last_steering_angle = current_angle
    self._initialized = True
    
    return True

  def monitor_longitudinal_safety(self, car_state: car.CarState, control_output: car.CarControl.ActuatorsOutput) -> bool:
    """
    Monitor longitudinal safety by checking for unexpected acceleration changes.
    
    Args:
      car_state: Current car state
      control_output: Control system output
      
    Returns:
      True if longitudinal control is safe, False otherwise
    """
    current_accel = car_state.aEgo
    self.acceleration_history.append(current_accel)
    
    if len(self.acceleration_history) >= 2 and self._initialized:
      # Calculate acceleration change rate (jerk)
      if len(self.acceleration_history) >= 2:
        recent_acceleration = list(self.acceleration_history)[-2:]
        if len(recent_acceleration) == 2:
          accel_change = abs(recent_acceleration[1] - recent_acceleration[0])
          jerk = accel_change / DT_CTRL  # Approximate jerk
      
          if jerk > self.max_acceleration_change:
            return False
    
    # Store current acceleration for next iteration
    self._last_acceleration = current_accel
    
    return True

  def monitor_model_prediction_safety(self, model_data) -> bool:
    """
    Monitor model prediction safety by checking for inconsistent or low-confidence predictions.
    
    Args:
      model_data: Model predictions from modeld
      
    Returns:
      True if model predictions are safe, False otherwise
    """
    if model_data is None:
      return True  # Can't assess, assume safe
    
    # Check for model confidence if available
    # This is a simplified check - in a real implementation, this would check
    # specific confidence metrics provided by the model
    try:
      if hasattr(model_data, 'meta') and hasattr(model_data.meta, 'engageProb'):
        # If engagement probability is too low, consider it unsafe
        if model_data.meta.engageProb < self.min_model_prediction_confidence:
          return False
    except (AttributeError, TypeError):
      pass
    
    # Check for sudden, unrealistic model predictions
    # Example: very high curvature that would be unachievable at current speed
    try:
      if (hasattr(model_data, 'action') and 
          hasattr(model_data.action, 'desiredCurvature') and 
          hasattr(model_data, 'carState')):
        v_ego = model_data.carState.vEgo if hasattr(model_data.carState, 'vEgo') else 0
        desired_curvature = model_data.action.desiredCurvature
        
        # Calculate max safe curvature based on speed (simplified)
        max_safe_curvature = 0.0 if v_ego < 1.0 else 4.0 / (v_ego * v_ego)  # Simplified safety check
        if abs(desired_curvature) > max_safe_curvature * 2.0:  # Allow some safety margin
          return False
    except (AttributeError, TypeError):
      pass
    
    return True

  def check_safety_violations(self, car_state: car.CarState, control_output: car.CarControl.ActuatorsOutput, model_data=None) -> tuple[bool, str]:
    """
    Perform comprehensive safety check and return whether the system is safe.
    
    Args:
      car_state: Current car state
      control_output: Control system output
      model_data: Model predictions (optional)
      
    Returns:
      Tuple of (is_safe, violation_reason)
    """
    if not self.safety_engaged:
      return True, "safety_disabled"
    
    # Check steering safety
    if not self.monitor_steering_safety(car_state, control_output):
      self.safety_violation_count += 1
      return False, "unsafe_steering"
    
    # Check longitudinal safety
    if not self.monitor_longitudinal_safety(car_state, control_output):
      self.safety_violation_count += 1
      return False, "unsafe_longitudinal"
    
    # Check model prediction safety
    if not self.monitor_model_prediction_safety(model_data):
      self.safety_violation_count += 1
      return False, "unsafe_model_predictions"
    
    # Reset violation count on successful safety check
    if self.safety_violation_count > 0:
      self.safety_violation_count = max(0, self.safety_violation_count - 0.1)  # Gradually decrease
    
    return True, "safe"

  def get_safety_recommendation(self, car_state: car.CarState, control_output: car.CarControl.ActuatorsOutput, model_data=None) -> log.SelfdriveState.AlertStatus:
    """
    Get safety-based alert recommendation for the UI.
    
    Args:
      car_state: Current car state
      control_output: Control system output
      model_data: Model predictions (optional)
      
    Returns:
      Alert status based on safety assessment
    """
    is_safe, violation_reason = self.check_safety_violations(car_state, control_output, model_data)
    
    if not is_safe:
      if self.safety_violation_count >= self.max_safety_violations:
        return log.SelfdriveState.AlertStatus.critical
      else:
        return log.SelfdriveState.AlertStatus.warning
    else:
      return log.SelfdriveState.AlertStatus.normal

  def should_disengage(self) -> bool:
    """
    Determine if the system should disengage based on safety violations.
    
    Returns:
      True if system should disengage, False otherwise
    """
    return self.safety_violation_count >= self.max_safety_violations

  def reset_safety_state(self):
    """Reset safety violation counters and state."""
    self.safety_violation_count = 0
    self._initialized = False