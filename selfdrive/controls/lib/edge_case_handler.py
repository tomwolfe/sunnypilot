"""
Edge case and unusual scenario handler for autonomous driving.

This module provides enhanced handling for various edge cases and unusual 
scenarios that can occur during autonomous driving to improve system robustness.
"""

from collections import deque
import numpy as np
from cereal import car, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter


class EdgeCaseHandler:
  """
  Handles various edge cases and unusual scenarios to improve system robustness.
  """
  
  def __init__(self):
    self.params = Params()
    
    # Filters for detecting unusual patterns
    self.lateral_accel_filter = FirstOrderFilter(0.0, 1.0, DT_CTRL)
    self.longitudinal_accel_filter = FirstOrderFilter(0.0, 1.0, DT_CTRL)
    self.steering_rate_filter = FirstOrderFilter(0.0, 0.5, DT_CTRL)
    
    # Tracking variables for edge case detection
    self.steering_angle_history = deque(maxlen=50)  # 0.5 seconds at 100Hz
    self.speed_history = deque(maxlen=50)  # 0.5 seconds at 100Hz
    self.lateral_accel_history = deque(maxlen=20)  # 0.2 seconds at 100Hz
    self.longitudinal_accel_history = deque(maxlen=20)  # 0.2 seconds at 100Hz
    
    # Edge case detection flags
    self._in_construction_zone = False
    self._in_tunnel = False  # Currently not implemented but reserved
    self._weather_conditions = "normal"  # Currently not implemented but reserved
    self._sharp_curve_detected = False
    self._sudden_object_detected = False
    self._abnormal_steering_detected = False
    
    # Configuration parameters
    self.max_lat_accel_threshold = 3.0  # m/s^2 - threshold for high lateral acceleration
    self.sharp_curve_speed_threshold = 15.0  # m/s - speed threshold for sharp curves
    self.steering_rate_threshold = 50.0  # deg/s - threshold for rapid steering changes
    self.speed_variance_threshold = 2.0  # m/s - threshold for speed variance detection
    self.construction_zone_speed = 10.0  # m/s - assumed construction zone speed limit
    
    # Initialize state variables
    self._last_steering_angle = 0.0
    self._last_speed = 0.0
    self._initialized = False
    
  def update_params(self):
    """Update edge case parameters from system parameters"""
    try:
      self.max_lat_accel_threshold = float(self.params.get("EdgeCaseMaxLatAccel", "3.0"))
      self.sharp_curve_speed_threshold = float(self.params.get("EdgeCaseSharpCurveSpeed", "15.0"))
      self.steering_rate_threshold = float(self.params.get("EdgeCaseSteeringRate", "50.0"))
      self.speed_variance_threshold = float(self.params.get("EdgeCaseSpeedVariance", "2.0"))
      self.construction_zone_speed = float(self.params.get("EdgeCaseConstructionSpeed", "10.0"))
    except (TypeError, ValueError):
      # Use defaults if parameters are invalid
      pass

  def detect_sharp_curves(self, car_state: car.CarState, model_data) -> bool:
    """
    Detect sharp curves and suggest appropriate speed reduction.
    
    Args:
      car_state: Current car state
      model_data: Model predictions
      
    Returns:
      True if sharp curve detected, False otherwise
    """
    if model_data is None or not hasattr(model_data, 'plan'):
      return False
    
    # Analyze the planned trajectory for sharp curves
    try:
      if hasattr(model_data.plan, 'orientationRate') and hasattr(model_data.plan.orientationRate, 'y'):
        # Check for high orientation rate (sharp turns)
        orientation_rates = model_data.plan.orientationRate.y[:10]  # Check first 10 points
        if len(orientation_rates) > 0:
          max_orientation_rate = max(abs(rate) for rate in orientation_rates)
          if max_orientation_rate > 0.2:  # Threshold for sharp curve
            return True
    except (AttributeError, TypeError):
      pass
    
    # Alternative: Check desired curvature from action
    if hasattr(model_data, 'action') and hasattr(model_data.action, 'desiredCurvature'):
      desired_curvature = abs(model_data.action.desiredCurvature)
      # For a vehicle, significant curvature might start at around 0.05/m
      if desired_curvature > 0.1 and car_state.vEgo > self.sharp_curve_speed_threshold:
        return True
    
    return False

  def detect_abnormal_steering(self, car_state: car.CarState) -> bool:
    """
    Detect abnormal steering behavior that might indicate road issues or construction.
    
    Args:
      car_state: Current car state
      
    Returns:
      True if abnormal steering detected, False otherwise
    """
    current_angle = car_state.steeringAngleDeg
    current_time = DT_CTRL  # We use DT_CTRL as delta time
    
    if self._initialized:
      # Calculate steering rate
      steering_rate = abs(current_angle - self._last_steering_angle) / current_time if current_time > 0 else 0
      self.steering_rate_filter.update(steering_rate)
      
      # Check for excessive steering rate
      if steering_rate > self.steering_rate_threshold:
        return True
      
      # Store in history for variance analysis
      self.steering_angle_history.append(current_angle)
      
      # Check for high variance in steering angles over short period
      if len(self.steering_angle_history) > 10:
        recent_angles = list(self.steering_angle_history)[-10:]
        angle_variance = np.var(recent_angles)
        if angle_variance > 50.0:  # High steering variance might indicate construction/rough road
          return True
    
    # Update tracking variables
    self._last_steering_angle = current_angle
    self._initialized = True
    
    return False

  def detect_sudden_objects(self, radar_state, model_data) -> bool:
    """
    Detect sudden objects that might require immediate action.
    
    Args:
      radar_state: Radar sensor data
      model_data: Model predictions
      
    Returns:
      True if sudden object detected, False otherwise
    """
    if radar_state is None:
      return False
    
    # Check for lead vehicles that suddenly appeared or changed behavior
    try:
      lead_one = radar_state.leadOne
      lead_two = radar_state.leadTwo
      
      if lead_one.status:
        # Check if lead vehicle is very close and relative velocity suggests potential collision
        if lead_one.dRel < 30.0 and lead_one.vRel < -2.0:  # Very close and approaching rapidly
          return True
          
      if lead_two.status:
        # Check second lead vehicle
        if lead_two.dRel < 50.0 and lead_two.vRel < -3.0:  # Within 50m and approaching rapidly
          return True
    except AttributeError:
      pass
    
    return False

  def detect_construction_zone(self, car_state: car.CarState, 
                              radar_state, model_data) -> bool:
    """
    Detect potential construction zones based on various indicators.
    
    Args:
      car_state: Current car state
      radar_state: Radar sensor data
      model_data: Model predictions
      
    Returns:
      True if construction zone detected, False otherwise
    """
    # Check for multiple indicators of construction zone:
    # 1. Unusual steering patterns
    # 2. Frequent speed changes
    # 3. Multiple vehicles at low speeds
    
    construction_indicators = 0
    
    # Check for abnormal steering (lane changes not by system)
    if self.detect_abnormal_steering(car_state):
      construction_indicators += 1
    
    # Check for speed variance
    current_speed = car_state.vEgo
    self.speed_history.append(current_speed)
    
    if len(self.speed_history) > 20:  # Wait for sufficient history
      recent_speeds = list(self.speed_history)[-20:]
      speed_variance = np.var(recent_speeds)
      if speed_variance > self.speed_variance_threshold:
        construction_indicators += 1
    
    # Check radar for multiple slow vehicles that might indicate construction
    if radar_state is not None:
      slow_vehicle_count = 0
      try:
        if radar_state.leadOne.status and radar_state.leadOne.vLead < 8.0:  # Below 28 km/h
          slow_vehicle_count += 1
        if radar_state.leadTwo.status and radar_state.leadTwo.vLead < 8.0:
          slow_vehicle_count += 1
          
        if slow_vehicle_count >= 1:  # At least one very slow vehicle
          construction_indicators += 1
      except AttributeError:
        pass
    
    # Additional model-based detection could go here
    # For example, detecting lane line patterns that are different from normal
    
    return construction_indicators >= 2  # At least 2 indicators to declare construction zone

  def handle_unusual_scenarios(self, car_state: car.CarState, 
                              radar_state=None, model_data=None) -> dict:
    """
    Analyze current situation and identify any unusual scenarios or edge cases.
    
    Args:
      car_state: Current car state
      radar_state: Radar sensor data (optional)
      model_data: Model predictions (optional)
      
    Returns:
      Dictionary containing detected scenarios and recommended actions
    """
    scenarios = {
      'sharp_curve': self.detect_sharp_curves(car_state, model_data),
      'construction_zone': self.detect_construction_zone(car_state, radar_state, model_data),
      'abnormal_steering': self.detect_abnormal_steering(car_state),
      'sudden_object': self.detect_sudden_objects(radar_state, model_data),
      'recommended_speed': car_state.vCruise * 0.8 if car_state.vCruise > 0 else car_state.vEgo,  # Default: reduce speed by 20%
      'caution_required': False,
      'adaptive_control_needed': False
    }
    
    # Set recommended speed based on detected scenarios
    if scenarios['sharp_curve'] and car_state.vEgo > self.sharp_curve_speed_threshold:
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], self.sharp_curve_speed_threshold)
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
      
    if scenarios['construction_zone']:
      scenarios['recommended_speed'] = min(scenarios['recommended_speed'], self.construction_zone_speed)
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
      
    if scenarios['sudden_object']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
      
    if scenarios['abnormal_steering']:
      scenarios['caution_required'] = True
      scenarios['adaptive_control_needed'] = True
    
    return scenarios
  
  def get_adaptive_control_modifications(self, car_state: car.CarState, 
                                       scenarios: dict) -> dict:
    """
    Get control modifications to adapt to detected edge cases and scenarios.
    
    Args:
      car_state: Current car state
      scenarios: Dictionary with detected scenarios from handle_unusual_scenarios
      
    Returns:
      Dictionary with control modifications
    """
    modifications = {
      'longitudinal_factor': 1.0,  # Factor to apply to longitudinal control (0.8 = 20% more conservative)
      'lateral_factor': 1.0,      # Factor to apply to lateral control
      'min_gap': 2.0,             # Minimum time gap to maintain
      'caution_mode': False       # Whether to enable caution mode
    }
    
    if scenarios['caution_required']:
      modifications['caution_mode'] = True
      modifications['longitudinal_factor'] = 0.8  # More conservative longitudinal control
      modifications['lateral_factor'] = 0.9      # More conservative lateral control
      modifications['min_gap'] = 3.0             # Increase minimum gap to 3 seconds
      
    if scenarios['sharp_curve']:
      modifications['lateral_factor'] = 0.7      # Extra conservative in curves
      modifications['longitudinal_factor'] = 0.7
      
    if scenarios['construction_zone']:
      modifications['longitudinal_factor'] = 0.6 # Extra conservative in construction zones
      modifications['lateral_factor'] = 0.8
      modifications['min_gap'] = 4.0             # Extra distance in construction zones
      
    if scenarios['sudden_object']:
      modifications['longitudinal_factor'] = 0.5 # Very conservative when objects detected
      modifications['min_gap'] = 5.0             # Maximum distance when sudden objects detected
    
    return modifications

  def reset_state(self):
    """Reset edge case detection state."""
    self.steering_angle_history.clear()
    self.speed_history.clear()
    self.lateral_accel_history.clear()
    self.longitudinal_accel_history.clear()
    self._initialized = False
    self._in_construction_zone = False
    self._sharp_curve_detected = False
    self._sudden_object_detected = False
    self._abnormal_steering_detected = False