"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import custom

from openpilot.common.constants import CV
from openpilot.common.params import Params

TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS


class NavigationE2EController:
  """
  Navigation-based E2E Turn Controller.
  Uses map data and navigation instructions to determine turn/lane change desires
  instead of relying solely on blinker signals.
  """

  TURN_LEFT = 1
  TURN_RIGHT = -1
  LANE_CHANGE_LEFT = 2
  LANE_CHANGE_RIGHT = -2
  NONE = 0

  def __init__(self):
    self.params = Params()
    self.enabled = self.params.get_bool("LaneTurnDesire")
    self.nav_desire = self.NONE
    self.turn_type = "none"
    self.distance_to_turn = float('inf')
    self.current_speed_limit = 0.0

    self.nav_enabled = self.params.get_bool("NavigationE2EEnabled")
    self.turn_speed_offset = 5.0 * CV.MPH_TO_MS

  def update_params(self):
    self.enabled = self.params.get_bool("LaneTurnDesire")
    self.nav_enabled = self.params.get_bool("NavigationE2EEnabled")

  def update_from_navigation(self, nav_instruction: dict, distance_to_maneuver: float,
                            current_speed_limit: float, road_edge_angle: float):
    """
    Update turn desire from navigation data.

    Args:
      nav_instruction: Navigation instruction type ('turn_left', 'turn_right',
                       'lane_change_left', 'lane_change_right', 'continue', 'none')
      distance_to_maneuver: Distance to the maneuver in meters
      current_speed_limit: Current speed limit in m/s
      road_edge_angle: Road edge angle for context
    """
    if not self.nav_enabled:
      self.nav_desire = self.NONE
      return

    self.distance_to_turn = distance_to_maneuver
    self.current_speed_limit = current_speed_limit
    self.turn_type = nav_instruction.get('type', 'none')

    if self.turn_type == 'turn_left':
      self.nav_desire = self.TURN_LEFT
    elif self.turn_type == 'turn_right':
      self.nav_desire = self.TURN_RIGHT
    elif self.turn_type == 'lane_change_left':
      self.nav_desire = self.LANE_CHANGE_LEFT
    elif self.turn_type == 'lane_change_right':
      self.nav_desire = self.LANE_CHANGE_RIGHT
    else:
      self.nav_desire = self.NONE

  def get_turn_desire(self, v_ego: float) -> int:
    """
    Get the navigation-based turn desire based on current conditions.

    Args:
      v_ego: Current vehicle speed

    Returns:
      Turn desire code (TURN_LEFT, TURN_RIGHT, LANE_CHANGE_LEFT, LANE_CHANGE_RIGHT, NONE)
    """
    if not self.nav_enabled or self.nav_desire == self.NONE:
      return self.NONE

    if self.distance_to_turn < 0 or self.distance_to_turn > 300:
      return self.NONE

    speed_limit_with_offset = self.current_speed_limit + self.turn_speed_offset
    if v_ego > speed_limit_with_offset:
      return self.NONE

    return self.nav_desire

  def get_turn_speed_limit(self) -> float:
    """Returns the speed limit to use during the maneuver."""
    return max(5.0 * CV.MPH_TO_MS, self.current_speed_limit - 5.0 * CV.MPH_TO_MS)

  def reset(self):
    self.nav_desire = self.NONE
    self.turn_type = "none"
    self.distance_to_turn = float('inf')


class LaneTurnController:
  def __init__(self, desire_helper):
    self.DH = desire_helper
    self.turn_direction = TurnDirection.none
    self.params = Params()
    self.lane_turn_value = float(self.params.get("LaneTurnValue", return_default=True)) * CV.MPH_TO_MS
    self.param_read_counter = 0
    self.enabled = self.params.get_bool("LaneTurnDesire")

    self.nav_controller = NavigationE2EController()

  def read_params(self):
    self.enabled = self.params.get_bool("LaneTurnDesire")
    value = float(self.params.get("LaneTurnValue", return_default=True)) * CV.MPH_TO_MS
    self.lane_turn_value = min(float(LANE_CHANGE_SPEED_MIN), value)
    self.nav_controller.update_params()

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_from_navigation(self, nav_instruction: dict, distance_to_maneuver: float,
                            current_speed_limit: float, road_edge_angle: float) -> None:
    """Update from navigation data for E2E navigation-based turns."""
    self.nav_controller.update_from_navigation(
      nav_instruction, distance_to_maneuver, current_speed_limit, road_edge_angle
    )

  def update_lane_turn(self, blindspot_left: bool, blindspot_right: bool,
                      left_blinker: bool, right_blinker: bool, v_ego: float) -> None:
    nav_desire = self.nav_controller.get_turn_desire(v_ego)

    if nav_desire == NavigationE2EController.TURN_LEFT:
      self.turn_direction = TurnDirection.turnLeft
    elif nav_desire == NavigationE2EController.TURN_RIGHT:
      self.turn_direction = TurnDirection.turnRight
    elif nav_desire == NavigationE2EController.LANE_CHANGE_LEFT:
      if v_ego < self.lane_turn_value and not blindspot_left:
        self.turn_direction = TurnDirection.turnLeft
      else:
        self.turn_direction = TurnDirection.none
    elif nav_desire == NavigationE2EController.LANE_CHANGE_RIGHT:
      if v_ego < self.lane_turn_value and not blindspot_right:
        self.turn_direction = TurnDirection.turnRight
      else:
        self.turn_direction = TurnDirection.none
    elif left_blinker and not right_blinker and v_ego < self.lane_turn_value and not blindspot_left:
      self.turn_direction = TurnDirection.turnLeft
    elif right_blinker and not left_blinker and v_ego < self.lane_turn_value and not blindspot_right:
      self.turn_direction = TurnDirection.turnRight
    else:
      self.turn_direction = TurnDirection.none

  def get_turn_direction(self):
    if not self.enabled:
      return TurnDirection.none
    return self.turn_direction
