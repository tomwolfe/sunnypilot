#!/usr/bin/env python3
"""
Navigation daemon for sunnypilot - Implements Tesla-style Navigate on Autopilot
Copyright (c) 2025, sunnypilot community
"""

import math
import time
from typing import List, Optional, Tuple

from cereal import messaging, log
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority

# Navigation update rate (1 Hz for route updates, faster for position tracking)
DT_NAV = 1.0
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.navd.helpers import Coordinate, distance_along_geometry, parse_banner_instructions
from openpilot.selfdrive.navd.safety import NavSafetyManager


class RouteEngine:
  """Handles route following logic with turn-by-turn navigation"""
  
  def __init__(self):
    self.coordinates: List[Coordinate] = []
    self.banners = []
    self.current_instruction: Optional[NavInstruction] = None
    self.route_valid = False
    self.distance_to_maneuver = 0.0
    self.total_route_distance = 0.0
    self.route_progress = 0.0
    
  def update_route(self, coordinates: List[Coordinate], banners: List[dict] = None):
    """Update route with new coordinates and banners"""
    self.coordinates = coordinates
    self.banners = banners or []
    self.route_valid = len(coordinates) > 1
    
    # Calculate total route distance
    if self.route_valid:
      total = 0.0
      for i in range(len(self.coordinates) - 1):
        total += self.coordinates[i].distance_to(self.coordinates[i+1])
      self.total_route_distance = total
  
  def update_position(self, current_pos: Coordinate) -> Optional[NavInstruction]:
    """Update position and return navigation instruction"""
    if not self.route_valid:
      return None
      
    # Calculate distance along route
    self.route_progress = distance_along_geometry(self.coordinates, current_pos)
    
    # Calculate distance to next maneuver
    if self.banners:
      # Find next maneuver by looking at distance along geometry
      for banner in sorted(self.banners, key=lambda x: x.get('distanceAlongGeometry', float('inf'))):
        banner_distance = banner.get('distanceAlongGeometry', 0.0)
        if banner_distance > self.route_progress:
          self.distance_to_maneuver = banner_distance - self.route_progress
          instruction = parse_banner_instructions([banner], self.distance_to_maneuver)
          if instruction:
            # Create NavInstruction object from parsed data
            self.current_instruction = NavInstruction(
              maneuver_type=instruction.get('maneuverType', ''),
              maneuver_modifier=instruction.get('maneuverModifier', ''),
              primary_text=instruction.get('maneuverPrimaryText', ''),
              secondary_text=instruction.get('maneuverSecondaryText', ''),
              distance_to_maneuver=self.distance_to_maneuver,
              show_full=instruction.get('showFull', False)
            )
            return self.current_instruction
          break
    
    return self.current_instruction


def calculate_eta(route_distance_remaining: float, current_speed: float) -> float:
  """Calculate estimated time of arrival in seconds"""
  if current_speed > 0.5:  # Avoid division by zero, use 1.8 km/h as threshold
    eta_seconds = route_distance_remaining / current_speed
  else:
    # If stopped, use average urban speed for remaining distance
    eta_seconds = route_distance_remaining / 15.0  # 15 m/s = ~54 km/h average
  return eta_seconds


def main():
  config_realtime_process(4, Priority.CTRL_MED)

  cloudlog.info("navd starting...")

  params = Params()
  route_engine = RouteEngine()
  safety_manager = NavSafetyManager()
  frame = 0

  # Set up messaging
  pm = messaging.PubMaster(['navInstruction', 'navRoute', 'mapRenderState', 'liveMapDataSP'])
  sm = messaging.SubMaster(['gpsLocation', 'carState', 'modelV2', 'selfdriveState'], poll='gpsLocation')

  # Current position
  current_pos = None
  last_pos = None
  current_speed = 0.0
  current_bearing = 0.0

  # Navigation context for DEC
  upcoming_turn_info = {
    'has_turn': False,
    'distance_to_turn': float('inf'),
    'turn_type': None,
    'turn_angle': 0.0
  }

  while True:
    sm.update()

    if sm.updated['gpsLocation']:
      gps = sm['gpsLocation']
      if gps.valid:
        current_pos = Coordinate(gps.latitude, gps.longitude)
        current_speed = gps.speed

        # Calculate bearing if we have previous position
        if last_pos is not None:
          bearing_delta = current_pos.latitude - last_pos.latitude
          current_bearing = math.atan2(current_pos.longitude - last_pos.longitude,
                                      bearing_delta) * 180.0 / math.pi
        last_pos = current_pos

        # Check safety before proceeding with navigation
        # Get the route position closest to current position for deviation check
        route_pos = route_engine.coordinates[0] if route_engine.coordinates else None
        if route_engine.route_valid and len(route_engine.coordinates) > 0:
          # Find the closest point on the route
          closest_route_pos = min(route_engine.coordinates,
                                 key=lambda coord: current_pos.distance_to(coord))
          route_pos = closest_route_pos

        # Check if navigation should be disengaged for safety
        if safety_manager.should_disengage_navigation(sm, current_pos, route_pos):
          cloudlog.warning("Navigation disengaged for safety reasons")
          # Clear navigation data to signal to UI and other processes
          route_engine.route_valid = False
          continue  # Skip the rest of this iteration

        # Update navigation with current position
        instruction = route_engine.update_position(current_pos)

        # Calculate ETA and navigation context for DEC
        if route_engine.route_valid:
          route_distance_remaining = max(0, route_engine.total_route_distance - route_engine.route_progress)
          eta_seconds = calculate_eta(route_distance_remaining, current_speed)

          # Publish navigation instruction
          if instruction is not None:
            # Publish navigation instruction
            msg = messaging.new_message('navInstruction')
            msg.valid = True

            nav_instruction = msg.navInstruction
            nav_instruction.maneuverType = instruction.maneuver_type
            nav_instruction.maneuverModifier = instruction.maneuver_modifier
            nav_instruction.maneuverPrimaryText = instruction.primary_text
            nav_instruction.maneuverSecondaryText = instruction.secondary_text
            nav_instruction.maneuverDistance = instruction.distance_to_maneuver  # Changed to maneuverDistance
            nav_instruction.showFull = instruction.show_full
            nav_instruction.distanceRemaining = route_distance_remaining
            nav_instruction.timeRemaining = eta_seconds
            nav_instruction.timeRemainingTypical = eta_seconds  # For now, same as current estimate

            pm.send('navInstruction', msg)

          # Publish map render state with ETA
          map_msg = messaging.new_message('mapRenderState')
          map_msg.valid = True

          map_state = map_msg.mapRenderState
          map_state.navValid = True

          pm.send('mapRenderState', map_msg)

        # Publish navigation context for DEC system
        # This provides route information to the DEC for smarter experimental mode decisions
        nav_context_msg = messaging.new_message('liveMapDataSP')
        nav_context_msg.valid = True

        live_map_data = nav_context_msg.liveMapDataSP
        live_map_data.navValid = route_engine.route_valid
        live_map_data.navStatus = 1 if route_engine.route_valid else 0  # 1 for active, 0 for inactive
        live_map_data.nextJunctionAngle = 0.0  # Will be calculated below
        live_map_data.distanceToNextJunction = float('inf')  # Will be set based on route

        # Set navigation context for upcoming turns for DEC
        if instruction and instruction.distance_to_maneuver < 200:  # Within 200m of maneuver
          live_map_data.distanceToNextJunction = instruction.distance_to_maneuver

          # Convert maneuver type to angle approximation
          if 'left' in instruction.maneuver_type.lower():
            live_map_data.nextJunctionAngle = -90.0  # Left turn
          elif 'right' in instruction.maneuver_type.lower():
            live_map_data.nextJunctionAngle = 90.0   # Right turn
          else:
            live_map_data.nextJunctionAngle = 0.0    # Straight

          # Update upcoming turn info for DEC integration
          upcoming_turn_info['has_turn'] = True
          upcoming_turn_info['distance_to_turn'] = instruction.distance_to_maneuver
          upcoming_turn_info['turn_type'] = instruction.maneuver_type
          upcoming_turn_info['turn_angle'] = live_map_data.nextJunctionAngle
        else:
          upcoming_turn_info['has_turn'] = False
          upcoming_turn_info['distance_to_turn'] = float('inf')
          live_map_data.distanceToNextJunction = float('inf')
          live_map_data.nextJunctionAngle = 0.0

        # Set turn info for DEC
        live_map_data.turnSpeedLimit = 0.0  # Will be calculated based on upcoming turns
        if upcoming_turn_info['has_turn'] and upcoming_turn_info['distance_to_turn'] < 100:
          # Calculate appropriate speed limit for upcoming turn
          # Use a simple formula: sharper turns need slower speeds
          angle_factor = abs(upcoming_turn_info['turn_angle']) / 90.0  # Normalize to 0-1 for 90-degree turns
          distance_factor = min(1.0, upcoming_turn_info['distance_to_turn'] / 50.0)  # Reduce speed as we get closer
          base_speed_limit = 15.0  # 15 m/s ~ 54 km/h
          turn_speed_limit = base_speed_limit * (1.0 - (angle_factor * distance_factor * 0.7))  # Reduce up to 70%
          live_map_data.turnSpeedLimit = max(5.0, turn_speed_limit)  # Minimum 5 m/s (18 km/h)

        pm.send('liveMapDataSP', nav_context_msg)

    # Check for route updates periodically
    if frame % int(2. / DT_NAV) == 0:  # Check every 2 seconds
      # In a real implementation, this would fetch updated route from external service
      # For now, check for route coordinates from params or external source
      route_coordinates_json = params.get("NavRouteCoordinates")
      if route_coordinates_json:
        # Parse route coordinates from JSON
        import json
        try:
          coords_data = json.loads(route_coordinates_json)
          coordinates = [Coordinate(c['lat'], c['lon']) for c in coords_data]
          route_engine.update_route(coordinates)
          # Reset safety state when new route is loaded
          safety_manager.reset_safety_state()
        except Exception as e:
          cloudlog.error(f"Error parsing route coordinates: {e}")

    frame += 1


if __name__ == "__main__":
  main()