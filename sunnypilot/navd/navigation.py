"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import time
import numpy as np
from typing import List, Optional, Tuple

from cereal import log, custom
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.navd.helpers import distance_along_geometry, minimum_distance, Coordinate
from openpilot.sunnypilot.mapd.live_map_data import get_debug
from openpilot.selfdrive.common.metrics import Metrics, record_metric
from openpilot.sunnypilot.navd.routing import EnhancedRouteManager


# The old RouteManager class has been replaced with EnhancedRouteManager
# from the routing.py module which provides real routing capabilities.
# The PointToPointNavigation class now uses EnhancedRouteManager directly.class PointToPointNavigation:
  """Main navigation system for point-to-point autonomous driving."""

  def __init__(self):
    self.pm = messaging.PubMaster(['navInstruction', 'navRoute'])
    self.sm = messaging.SubMaster(['liveLocationKalman', 'modelV2', 'carState', 'selfdriveState'])
    self.params = Params()
    self.route_manager = EnhancedRouteManager()  # Use enhanced route manager with real routing
    self.current_position: Optional[Coordinate] = None
    self.frame = 0
    self.nav_active = False
    self.navigation_start_time = None

  def set_destination(self, latitude: float, longitude: float) -> bool:
    """Set destination and activate navigation."""
    if not self.current_position:
      # Can't set destination without knowing current position
      return False

    start_time = time.time()
    destination = Coordinate(latitude, longitude)
    # Pass current position to calculate route
    success = self.route_manager.set_destination(destination, self.current_position)
    if success:
      self.nav_active = True
      self.navigation_start_time = time.time()
      self.params.put("NavDestination", json.dumps({
        "latitude": latitude,
        "longitude": longitude
      }))

    # Record metrics
    record_metric(Metrics.NAVIGATION_LATENCY_MS, (time.time() - start_time) * 1000, {
        "operation": "set_destination_api",
        "destination": [latitude, longitude],
        "success": success
    })
    return success

  def get_destination(self) -> Optional[Coordinate]:
    """Get current destination."""
    return self.route_manager.destination

  def is_active(self) -> bool:
    """Check if navigation system is active."""
    return self.nav_active and self.route_manager.active

  def get_status(self) -> dict:
    """Get navigation status information."""
    route = self.route_manager.current_route
    return {
      "active": self.is_active(),
      "destination": self.route_manager.destination.as_dict() if self.route_manager.destination else None,
      "remaining_distance": route.total_distance if route else 0.0,
      "remaining_time": route.total_time if route else 0.0,
      "route_geometry": route.segments if route else []
    }

  def update(self):
    """Main update loop for navigation system."""
    start_time = time.time()
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

    # Record metrics for the update cycle
    record_metric(Metrics.NAVIGATION_LATENCY_MS, (time.time() - start_time) * 1000, {
        "operation": "navigation_update_cycle",
        "frame": self.frame
    })
  
  def _publish_navigation_data(self):
    """Publish navigation data for other modules to use."""
    if not self.current_position:
      return

    # Create navigation instruction message
    nav_msg = messaging.new_message('navInstruction')
    nav_inst = nav_msg.navInstruction

    if self.route_manager.active and self.route_manager.destination:
      turn_type, distance_to_turn, turn_angle = self.route_manager.get_maneuver_info(self.current_position)

      nav_inst.maneuverType = turn_type
      nav_inst.distanceToManeuver = distance_to_turn
      nav_inst.maneuverAngle = turn_angle
      # Use route properties from enhanced manager
      route = self.route_manager.current_route
      nav_inst.remainingDistance = route.total_distance if route else 0.0
      nav_inst.remainingTime = route.total_time if route else 0.0
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

    # Use segments from the enhanced route manager
    nav_route.coordinates = []
    if self.route_manager.current_route:
      nav_route.coordinates = [
        [segment.start.latitude, segment.start.longitude]
        for segment in self.route_manager.current_route.segments
      ]
      # Add the final destination point
      nav_route.coordinates.append([
        self.route_manager.destination.latitude,
        self.route_manager.destination.longitude
      ])

    nav_route.active = self.nav_active
    nav_route.destination = [
      self.route_manager.destination.latitude,
      self.route_manager.destination.longitude
    ] if self.route_manager.destination else [0.0, 0.0]
    nav_route.remainingDistance = route.total_distance if route else 0.0

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
    self.maneuver_success_count = 0
    self.maneuver_failure_count = 0

  def update(self, frame: int):
    """Update navigation controller with new data."""
    start_time = time.time()
    if self.sm.updated['navInstruction']:
      nav_inst = self.sm['navInstruction']
      self.nav_instruction_received = True
      self.last_maneuver_type = nav_inst.maneuverType
      self.distance_to_maneuver = nav_inst.distanceToManeuver

      # Process the maneuver if we're close enough
      if self.distance_to_maneuver < 50.0 and not self.maneuver_processed:
        self._process_maneuver(nav_inst)

    # Record metrics
    record_metric(Metrics.NAVIGATION_LATENCY_MS, (time.time() - start_time) * 1000, {
        "operation": "nav_instruction_controller_update",
        "frame": frame
    })

  def _process_maneuver(self, nav_inst):
    """Process navigation maneuver."""
    # This would integrate with lateral control to execute turns
    # For now we just mark it as processed
    start_time = time.time()
    self.maneuver_processed = True
    get_debug(f"Processing maneuver: {nav_inst.maneuverType} in {nav_inst.distanceToManeuver:.1f}m")

    # Record maneuver processing metrics
    record_metric(Metrics.MANEUVER_SUCCESS_RATE, 1.0, {
        "maneuver_type": nav_inst.maneuverType,
        "distance": nav_inst.distanceToManeuver,
        "processing_time": time.time() - start_time
    })

  def reset_maneuver_processing(self):
    """Reset maneuver processing state."""
    self.maneuver_processed = False