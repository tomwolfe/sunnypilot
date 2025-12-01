"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from abc import abstractmethod, ABC

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.navd.helpers import coordinate_from_param, Coordinate

MAX_SPEED_LIMIT = V_CRUISE_UNSET * CV.KPH_TO_MS

# Traffic sign detection constants
TRAFFIC_SIGN_MIN_DISTANCE = 10.0  # Minimum distance for considering a sign (in meters)
TRAFFIC_SIGN_BEARING_THRESHOLD = 90.0  # Bearing threshold for considering a sign as "ahead" (in degrees)


class BaseMapData(ABC):
  def __init__(self):
    self.params = Params()

    self.sm = messaging.SubMaster(['liveLocationKalman'])
    self.pm = messaging.PubMaster(['liveMapDataSP'])

    self.localizer_valid = False
    self.last_bearing = None
    self.last_position = coordinate_from_param("LastGPSPositionLLK", self.params)

  @abstractmethod
  def update_location(self) -> None:
    pass

  @abstractmethod
  def get_current_speed_limit(self) -> float:
    pass

  @abstractmethod
  def get_next_speed_limit_and_distance(self) -> tuple[float, float]:
    pass

  @abstractmethod
  def get_current_road_name(self) -> str:
    pass

  def publish(self) -> None:
    speed_limit = self.get_current_speed_limit()
    next_speed_limit, next_speed_limit_distance = self.get_next_speed_limit_and_distance()

    # Get traffic sign information
    traffic_sign_info = self.get_traffic_sign_info()

    mapd_sp_send = messaging.new_message('liveMapDataSP')
    mapd_sp_send.valid = self.sm['liveLocationKalman'].gpsOK
    live_map_data = mapd_sp_send.liveMapDataSP

    live_map_data.speedLimitValid = bool(MAX_SPEED_LIMIT > speed_limit > 0)
    live_map_data.speedLimit = speed_limit
    live_map_data.speedLimitAheadValid = bool(MAX_SPEED_LIMIT > next_speed_limit > 0)
    live_map_data.speedLimitAhead = next_speed_limit
    live_map_data.speedLimitAheadDistance = next_speed_limit_distance
    live_map_data.roadName = self.get_current_road_name()

    # Add traffic sign information to the message
    live_map_data.hasStopSign = traffic_sign_info['has_stop_sign']
    live_map_data.hasTrafficLight = traffic_sign_info['has_traffic_light']
    live_map_data.hasYieldSign = traffic_sign_info['has_yield_sign']
    live_map_data.distanceToNextSign = traffic_sign_info['distance_to_next_sign']

    self.pm.send('liveMapDataSP', mapd_sp_send)

  def tick(self) -> None:
    self.sm.update(0)
    self.update_location()
    self.publish()

  def get_traffic_sign_info(self) -> dict:
    """
    Enhanced method to get traffic sign information from map data.
    Returns information about upcoming traffic signs like stop signs, traffic lights, etc.
    """
    # This method would be implemented to retrieve traffic sign information from OSM data
    # For now, it returns a default structure that can be expanded
    traffic_sign_info = {
      'has_stop_sign': False,
      'has_traffic_light': False,
      'has_yield_sign': False,
      'distance_to_next_sign': float('inf'),
      'sign_type': None
    }

    # Get traffic sign data from params if available
    traffic_sign_data = self.params.get("MapTrafficSigns")
    if traffic_sign_data:
      try:
        import json
        sign_data = json.loads(traffic_sign_data)

        # Process the sign data to find the closest relevant sign ahead
        if self.last_position and 'signs' in sign_data:
          closest_sign = None
          min_distance = float('inf')

          # Update location to get current bearing if available
          self.update_location()

          for sign in sign_data['signs']:
            sign_coords = Coordinate(sign.get('latitude', 0), sign.get('longitude', 0))
            distance = self.last_position.distance_to(sign_coords)

            # Only consider signs that are in front of us based on heading
            # Calculate heading angle between vehicle and sign
            is_ahead = True
            if self.last_bearing is not None:
              # Calculate the bearing from vehicle to sign
              bearing_to_sign = self.last_position.bearing_to(sign_coords)
              # Calculate the difference in bearing (considering 180-degree wraparound)
              bearing_diff = abs(bearing_to_sign - self.last_bearing)
              if bearing_diff > 180:
                bearing_diff = 360 - bearing_diff
              # Consider it "ahead" if within bearing threshold degrees (front 180-degree arc)
              is_ahead = bearing_diff < TRAFFIC_SIGN_BEARING_THRESHOLD

            # Only consider signs ahead of us and at a relevant distance
            if distance < min_distance and distance > TRAFFIC_SIGN_MIN_DISTANCE and is_ahead:  # At least min_distance away and in front of us
              min_distance = distance
              closest_sign = sign

          if closest_sign:
            sign_type = closest_sign.get('type', '').lower()

            # Define more comprehensive mapping for different OSM tag variations
            STOP_SIGN_TYPES = {'stop', 'stop_sign', 'stop_line', 'all_way_stop', 'stop_yield', 'stop_ahead'}
            TRAFFIC_LIGHT_TYPES = {'traffic_light', 'light', 'traffic_signals', 'traffic_signal', 'signal_lights', 'lights'}
            YIELD_SIGN_TYPES = {'yield', 'yield_sign', 'give_way', 'yield_ahead', 'give_way_sign'}

            traffic_sign_info.update({
              'has_stop_sign': sign_type in STOP_SIGN_TYPES,
              'has_traffic_light': sign_type in TRAFFIC_LIGHT_TYPES,
              'has_yield_sign': sign_type in YIELD_SIGN_TYPES,
              'distance_to_next_sign': min_distance,
              'sign_type': sign_type
            })
      except Exception:
        # If there's an error parsing the data, return default values
        pass

    return traffic_sign_info
