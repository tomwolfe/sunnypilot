"""
Navigation Safety Module for sunnypilot - Safety checks and fallback mechanisms for Navigate on Autopilot
Copyright (c) 2025, sunnypilot community
"""
from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
import time


class NavSafetyManager:
  """Manages safety checks and fallback mechanisms for navigation system"""
  
  def __init__(self):
    self.params = Params()
    self.last_valid_nav_time = 0
    self.nav_timeout_threshold = 30.0  # seconds before considering navigation invalid
    self.route_loss_threshold = 10.0  # seconds before route is considered lost
    self.last_position_update = None
    self.route_following_active = False
    self.safety_disengaged = False
    
    # Statistics for safety monitoring
    self.route_deviation_counter = 0
    self.max_deviation_distance = 50.0  # meters before considering off-route
    self.deviation_reset_threshold = 5.0  # meters to reset deviation counter
    
  def update_position(self, current_pos, route_pos=None):
    """Update current position and check for route adherence"""
    if route_pos is None:
      return True
      
    # Calculate distance from route
    distance_from_route = current_pos.distance_to(route_pos)
    
    # Update deviation counter
    if distance_from_route > self.max_deviation_distance:
      self.route_deviation_counter += 1
      cloudlog.warning(f"Navigation deviation detected: {distance_from_route:.1f}m from route")
    elif distance_from_route < self.deviation_reset_threshold:
      # Reset counter when back on route
      self.route_deviation_counter = max(0, self.route_deviation_counter - 1)
    
    # Check if we're significantly off route
    if self.route_deviation_counter > 5:  # 5 consecutive checks off route
      cloudlog.error("Vehicle significantly off route, disengaging navigation mode")
      self.safety_disengaged = True
      return False
      
    return True
  
  def check_route_validity(self, sm: messaging.SubMaster):
    """Check if navigation route is still valid"""
    current_time = time.monotonic()
    
    # Check if nav instruction has been updated recently
    if sm.updated['navInstruction']:
      self.last_valid_nav_time = current_time
    
    # Check for timeout
    time_since_last_valid = current_time - self.last_valid_nav_time
    if time_since_last_valid > self.nav_timeout_threshold:
      cloudlog.warning(f"Navigation data timeout: {time_since_last_valid:.1f}s since last update")
      self.safety_disengaged = True
      return False
    
    return True
  
  def check_speed_compliance(self, current_speed, speed_limit=None, turn_speed_limit=None):
    """Check if current speed is appropriate for navigation context"""
    if speed_limit is not None and current_speed > speed_limit:
      # Allow some tolerance for speed limit compliance
      tolerance = 2.0  # 2 m/s tolerance
      if current_speed > speed_limit + tolerance:
        cloudlog.warning(f"Exceeding navigation speed limit: {current_speed:.1f} > {speed_limit:.1f}")
        return False
    
    if turn_speed_limit is not None and current_speed > turn_speed_limit:
      # Approaching turn, should slow down
      if current_speed > turn_speed_limit:
        cloudlog.warning(f"Too fast for upcoming turn: {current_speed:.1f} > {turn_speed_limit:.1f}")
        return False
    
    return True
  
  def check_navigation_conditions(self, sm: messaging.SubMaster, current_pos, route_pos=None):
    """Comprehensive navigation safety check"""
    # Check route validity
    if not self.check_route_validity(sm):
      return False
    
    # Check route adherence if route position is provided
    if route_pos is not None:
      if not self.update_position(current_pos, route_pos):
        return False
    
    # Check speed compliance with navigation context
    car_state = sm['carState']
    live_map_data = sm.get('liveMapDataSP')
    
    speed_compliant = self.check_speed_compliance(
      car_state.vEgo,
      getattr(live_map_data, 'speedLimit', None),
      getattr(live_map_data, 'turnSpeedLimit', None)
    )
    
    if not speed_compliant:
      return False
    
    # Additional safety checks can be added here
    # For example, checking if in experimental mode is appropriate for navigation
    self.safety_disengaged = False
    return True
  
  def should_disengage_navigation(self, sm: messaging.SubMaster, current_pos, route_pos=None):
    """Determine if navigation should be disengaged for safety reasons"""
    return not self.check_navigation_conditions(sm, current_pos, route_pos)
  
  def reset_safety_state(self):
    """Reset safety state when navigation is re-enabled"""
    self.route_deviation_counter = 0
    self.safety_disengaged = False
    self.last_valid_nav_time = time.monotonic()
