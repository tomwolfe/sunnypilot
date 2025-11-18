#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import time
from typing import Optional

from cereal import log
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MAX_VALS, A_CRUISE_MAX_BP
from openpilot.sunnypilot.navd.navigation import PointToPointNavigation, NavInstructionController
from openpilot.sunnypilot.navd.helpers import Coordinate
from openpilot.selfdrive.common.hardware_monitor import start_hardware_monitoring


class NavDaemon:
  """Main navigation daemon that handles destination setting and route management."""
  
  def __init__(self):
    config_realtime_process(6, 5)  # Configure for appropriate priority
    
    self.params = Params()
    self.navigation = PointToPointNavigation()
    self.nav_controller = NavInstructionController()
    
    # Messaging
    self.pm = messaging.PubMaster(['navStatus', 'navDestination'])
    self.sm = messaging.SubMaster([
      'liveLocationKalman', 
      'modelV2', 
      'carState', 
      'selfdriveState',
      'navInstruction'
    ])
    
    # State tracking
    self.destination_set = False
    self.last_destination_request = 0
    self.frame = 0
    
    cloudlog.info("NavDaemon initialized")

  def _handle_destination_requests(self):
    """Handle destination requests from params."""
    # Check if there's a new destination request
    nav_destination_str = self.params.get("NavDestinationRequest")
    if nav_destination_str and time.time() - self.last_destination_request > 1.0:
      try:
        nav_request = json.loads(nav_destination_str)
        if 'latitude' in nav_request and 'longitude' in nav_request:
          dest_lat = float(nav_request['latitude'])
          dest_lon = float(nav_request['longitude'])
          
          success = self.navigation.set_destination(dest_lat, dest_lon)
          if success:
            self.destination_set = True
            cloudlog.info(f"Destination set: {dest_lat}, {dest_lon}")
            # Clear the request param
            self.params.delete("NavDestinationRequest")
            self.last_destination_request = time.time()
          else:
            cloudlog.error("Failed to set destination")
      except (json.JSONDecodeError, ValueError, KeyError) as e:
        cloudlog.error(f"Error parsing destination request: {e}")
        self.params.delete("NavDestinationRequest")

  def _publish_nav_status(self):
    """Publish navigation status."""
    nav_status_msg = messaging.new_message('navStatus')
    nav_status = nav_status_msg.navStatus
    
    status = self.navigation.get_status()
    nav_status.active = status['active']
    nav_status.remainingDistance = status['remaining_distance']
    nav_status.remainingTime = status['remaining_time']
    nav_status.destination = status['destination'] or [0.0, 0.0]
    
    self.pm.send('navStatus', nav_status_msg)

  def _publish_destination_info(self):
    """Publish destination information."""
    dest_msg = messaging.new_message('navDestination')
    dest_info = dest_msg.navDestination
    
    current_dest = self.navigation.get_destination()
    if current_dest:
      dest_info.latitude = current_dest.latitude
      dest_info.longitude = current_dest.longitude
      dest_info.active = True
    else:
      dest_info.latitude = 0.0
      dest_info.longitude = 0.0
      dest_info.active = False
    
    self.pm.send('navDestination', dest_msg)

  def update(self):
    """Main update loop."""
    self.sm.update(0)
    
    # Handle new destination requests
    self._handle_destination_requests()
    
    # Update navigation system
    self.navigation.update()
    
    # Update navigation controller
    self.nav_controller.update(self.frame)
    
    # Publish status information
    if self.frame % 5 == 0:  # Publish every 5 frames (~10Hz)
      self._publish_nav_status()
      self._publish_destination_info()
    
    self.frame += 1

  def run(self):
    """Run the navigation daemon."""
    cloudlog.info("NavDaemon starting")
    while True:
      try:
        self.update()
        time.sleep(DT_MDL)  # Sleep for model delay time
      except Exception as e:
        cloudlog.exception(f"NavDaemon error: {e}")
        time.sleep(1.0)


def main():
  # Start hardware monitoring
  start_hardware_monitoring()

  nav_daemon = NavDaemon()
  nav_daemon.run()


if __name__ == "__main__":
  main()