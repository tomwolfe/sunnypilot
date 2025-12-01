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

    mapd_sp_send = messaging.new_message('liveMapDataSP')
    mapd_sp_send.valid = self.sm['liveLocationKalman'].gpsOK
    live_map_data = mapd_sp_send.liveMapDataSP

    live_map_data.speedLimitValid = bool(MAX_SPEED_LIMIT > speed_limit > 0)
    live_map_data.speedLimit = speed_limit
    live_map_data.speedLimitAheadValid = bool(MAX_SPEED_LIMIT > next_speed_limit > 0)
    live_map_data.speedLimitAhead = next_speed_limit
    live_map_data.speedLimitAheadDistance = next_speed_limit_distance
    live_map_data.roadName = self.get_current_road_name()

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
      'sign_type': None,
      'sign_coordinates': None
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

          for sign in sign_data['signs']:
            sign_coords = Coordinate(sign.get('latitude', 0), sign.get('longitude', 0))
            distance = self.last_position.distance_to(sign_coords)

            # Only consider signs ahead of us (simple heuristic based on distance)
            if distance < min_distance and distance > 10:  # At least 10m away to be relevant
              min_distance = distance
              closest_sign = sign

          if closest_sign:
            sign_type = closest_sign.get('type', '').lower()
            traffic_sign_info.update({
              'has_stop_sign': sign_type == 'stop',
              'has_traffic_light': sign_type in ['traffic_light', 'light'],
              'has_yield_sign': sign_type == 'yield',
              'distance_to_next_sign': min_distance,
              'sign_type': sign_type,
              'sign_coordinates': sign_coords
            })
      except Exception:
        # If there's an error parsing the data, return default values
        pass

    return traffic_sign_info
