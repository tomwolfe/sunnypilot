"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import numpy as np
from typing import List, Optional, Tuple

from cereal import log, custom
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.navd.helpers import distance_along_geometry, minimum_distance, Coordinate
from openpilot.sunnypilot.mapd.live_map_data import get_debug


class RouteManager:
  """Manages navigation routes and provides turn-by-turn guidance."""
  
  def __init__(self):
    self.params = Params()
    self.destination: Optional[Coordinate] = None
    self.route_geometry: List[Coordinate] = []
    self.route_instructions = []
    self.route_distance = 0.0
    self.route_remaining_distance = 0.0
    self.route_remaining_time = 0.0
    self.active = False
    self.current_step_index = 0
    self.turning_threshold = 30.0  # meters before turn to trigger alert
    
  def set_destination(self, coordinate: Coordinate) -> bool:
    """Set a new destination and request route calculation."""
    self.destination = coordinate
    # In a real implementation, this would call external routing service
    # For now, we'll simulate a basic route
    self.active = True
    self.current_step_index = 0
    get_debug(f"Destination set to: {coordinate.latitude}, {coordinate.longitude}")
    return True
    
  def update_route(self, current_pos: Coordinate) -> bool:
    """Update route information based on current position."""
    if not self.active or not self.destination:
      return False
      
    # Calculate distance to destination
    distance_to_dest = current_pos.distance_to(self.destination)
    self.route_remaining_distance = distance_to_dest
    
    # For simulation purposes, estimate time based on average speed
    # In reality, this would come from routing service
    avg_speed = 15.0  # m/s (about 54 km/h)
    self.route_remaining_time = distance_to_dest / avg_speed if avg_speed > 0 else 0
    
    # Check if we need to trigger turn guidance
    self._update_turn_guidance(current_pos)
    
    return True

  def _update_turn_guidance(self, current_pos: Coordinate):
    """Update turn guidance based on route geometry."""
    # This would be more sophisticated with real route data
    pass

  def get_maneuver_info(self) -> Tuple[str, float, float]:
    """Get the next maneuver information."""
    if not self.active or not self.destination:
      return "none", 0.0, 0.0
      
    # Return turn type, distance to turn, and turn angle
    # In a real implementation, this would come from route steps
    distance_to_dest = 0.0
    if self.route_geometry:
      distance_to_dest = minimum_distance(
        self.route_geometry[0], 
        self.route_geometry[-1], 
        current_pos
      )
    else:
      # Simple straight-line distance if no route geometry
      distance_to_dest = current_pos.distance_to(self.destination)
    
    return "arrive", distance_to_dest, 0.0  # arrival maneuver

  def check_route_completion(self, current_pos: Coordinate, tolerance: float = 50.0) -> bool:
    """Check if we have reached the destination."""
    if not self.active or not self.destination:
      return False
      
    distance = current_pos.distance_to(self.destination)
    reached = distance <= tolerance
    if reached:
      self.active = False
      get_debug(f"Destination reached! Distance: {distance:.2f}m")
    return reached

  def clear_route(self):
    """Clear the current route."""
    self.destination = None
    self.route_geometry = []
    self.route_instructions = []
    self.route_distance = 0.0
    self.route_remaining_distance = 0.0
    self.route_remaining_time = 0.0
    self.active = False
    self.current_step_index = 0
    get_debug("Route cleared")


class PointToPointNavigation:
  """Main navigation system for point-to-point autonomous driving."""
  
  def __init__(self):
    self.pm = messaging.PubMaster(['navInstruction', 'navRoute'])
    self.sm = messaging.SubMaster(['liveLocationKalman', 'modelV2', 'carState', 'selfdriveState'])
    self.params = Params()
    self.route_manager = RouteManager()
    self.current_position: Optional[Coordinate] = None
    self.frame = 0
    self.nav_active = False
    
  def set_destination(self, latitude: float, longitude: float) -> bool:
    """Set destination and activate navigation."""
    destination = Coordinate(latitude, longitude)
    success = self.route_manager.set_destination(destination)
    if success:
      self.nav_active = True
      self.params.put("NavDestination", json.dumps({
        "latitude": latitude,
        "longitude": longitude
      }))
    return success

  def get_destination(self) -> Optional[Coordinate]:
    """Get current destination."""
    return self.route_manager.destination

  def is_active(self) -> bool:
    """Check if navigation system is active."""
    return self.nav_active and self.route_manager.active

  def get_status(self) -> dict:
    """Get navigation status information."""
    return {
      "active": self.is_active(),
      "destination": self.route_manager.destination.as_dict() if self.route_manager.destination else None,
      "remaining_distance": self.route_manager.route_remaining_distance,
      "remaining_time": self.route_manager.route_remaining_time,
      "route_geometry": [c.as_dict() for c in self.route_manager.route_geometry]
    }

  def update(self):
    """Main update loop for navigation system."""
    self.sm.update(0)
    
    # Update current position from localization
    location = self.sm['liveLocationKalman']
    if location.positionGeodetic.valid:
      self.current_position = Coordinate(
        location.positionGeodetic.value[0],  # latitude
        location.positionGeodetic.value[1]  # longitude
      )
    
    if self.current_position and self.route_manager.active:
      # Update route information
      self.route_manager.update_route(self.current_position)
      
      # Check if we've reached destination
      if self.route_manager.check_route_completion(self.current_position):
        self.nav_active = False
        self._send_arrival_event()
    
    # Send navigation information
    self._publish_navigation_data()
    
    self.frame += 1
  
  def _publish_navigation_data(self):
    """Publish navigation data for other modules to use."""
    if not self.current_position:
      return
      
    # Create navigation instruction message
    nav_msg = messaging.new_message('navInstruction')
    nav_inst = nav_msg.navInstruction
    
    if self.route_manager.active and self.route_manager.destination:
      turn_type, distance_to_turn, turn_angle = self.route_manager.get_maneuver_info()
      
      nav_inst.maneuverType = turn_type
      nav_inst.distanceToManeuver = distance_to_turn
      nav_inst.maneuverAngle = turn_angle
      nav_inst.remainingDistance = self.route_manager.route_remaining_distance
      nav_inst.remainingTime = self.route_manager.route_remaining_time
      nav_inst.active = self.nav_active
      nav_inst.coordinates = [
        self.current_position.latitude,
        self.current_position.longitude,
        self.route_manager.destination.latitude,
        self.route_manager.destination.longitude
      ]
    
    # Create navigation route message
    route_msg = messaging.new_message('navRoute')
    nav_route = route_msg.navRoute
    
    nav_route.coordinates = [
      [c.latitude, c.longitude] for c in self.route_manager.route_geometry
    ]
    nav_route.active = self.nav_active
    nav_route.destination = [
      self.route_manager.destination.latitude,
      self.route_manager.destination.longitude
    ] if self.route_manager.destination else [0.0, 0.0]
    nav_route.remainingDistance = self.route_manager.route_remaining_distance
    
    # Send messages
    self.pm.send('navInstruction', nav_msg)
    self.pm.send('navRoute', route_msg)
  
  def _send_arrival_event(self):
    """Send arrival event when destination is reached."""
    pass  # In a real implementation, this would send appropriate events

  def deactivate(self):
    """Deactivate navigation system."""
    self.nav_active = False
    self.route_manager.clear_route()
    self.params.delete("NavDestination")


class NavInstructionController:
  """Integrates navigation instructions with lateral control."""
  
  def __init__(self):
    self.sm = messaging.SubMaster(['navInstruction', 'modelV2', 'carState'])
    self.nav_instruction_received = False
    self.last_maneuver_type = "none"
    self.distance_to_maneuver = float('inf')
    self.maneuver_processed = False

  def update(self, frame: int):
    """Update navigation controller with new data."""
    if self.sm.updated['navInstruction']:
      nav_inst = self.sm['navInstruction']
      self.nav_instruction_received = True
      self.last_maneuver_type = nav_inst.maneuverType
      self.distance_to_maneuver = nav_inst.distanceToManeuver
      
      # Process the maneuver if we're close enough
      if self.distance_to_maneuver < 50.0 and not self.maneuver_processed:
        self._process_maneuver(nav_inst)
  
  def _process_maneuver(self, nav_inst):
    """Process navigation maneuver."""
    # This would integrate with lateral control to execute turns
    # For now we just mark it as processed
    self.maneuver_processed = True
    get_debug(f"Processing maneuver: {nav_inst.maneuverType} in {nav_inst.distanceToManeuver:.1f}m")

  def reset_maneuver_processing(self):
    """Reset maneuver processing state."""
    self.maneuver_processed = False