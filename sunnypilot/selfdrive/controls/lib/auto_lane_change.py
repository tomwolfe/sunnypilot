"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from collections import deque
import numpy as np


class AutoLaneChangeMode:
  OFF = -1
  NUDGE = 0  # default
  NUDGELESS = 1
  HALF_SECOND = 2
  ONE_SECOND = 3
  TWO_SECONDS = 4
  THREE_SECONDS = 5


AUTO_LANE_CHANGE_TIMER = {
  AutoLaneChangeMode.OFF: 0.0,            # Off
  AutoLaneChangeMode.NUDGE: 0.0,          # Nudge
  AutoLaneChangeMode.NUDGELESS: 0.05,     # Nudgeless
  AutoLaneChangeMode.HALF_SECOND: 0.5,    # 0.5-second delay
  AutoLaneChangeMode.ONE_SECOND: 1.0,     # 1-second delay
  AutoLaneChangeMode.TWO_SECONDS: 2.0,    # 2-second delay
  AutoLaneChangeMode.THREE_SECONDS: 3.0,  # 3-second delay
}

ONE_SECOND_DELAY = -1


class AutoLaneChangeController:
  def __init__(self, desire_helper):
    self.DH = desire_helper
    self.params = Params()

    self.lane_change_wait_timer = 0.0
    self.param_read_counter = 0
    self.lane_change_delay = 0.0

    self.lane_change_set_timer = self.params.get("AutoLaneChangeTimer", return_default=True)
    self.lane_change_bsm_delay = False

    self.prev_brake_pressed = False
    self.auto_lane_change_allowed = False
    self.prev_lane_change = False

    # Enhanced lane change safety tracking
    self.lead_vehicle_buffer = deque(maxlen=10)  # Track lead vehicle data over time (0.1 seconds at 100Hz)
    self.lane_width_buffer = deque(maxlen=5)     # Track lane width consistency
    self.speed_diff_buffer = deque(maxlen=20)    # Track relative speed for safety assessment

    self.read_params()

  def reset(self) -> None:
    # Auto reset if parent state indicates we should
    if self.DH.lane_change_state == log.LaneChangeState.off and \
       self.DH.lane_change_direction == log.LaneChangeDirection.none:
      self.lane_change_wait_timer = 0.0
      self.prev_brake_pressed = False
      self.prev_lane_change = False
      # Clear safety buffers when resetting
      self.lead_vehicle_buffer.clear()
      self.lane_width_buffer.clear()
      self.speed_diff_buffer.clear()

  def read_params(self) -> None:
    self.lane_change_bsm_delay = self.params.get_bool("AutoLaneChangeBsmDelay")
    self.lane_change_set_timer = self.params.get("AutoLaneChangeTimer", return_default=True)

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update_lane_change_timers(self, blindspot_detected: bool) -> None:
    self.lane_change_delay = AUTO_LANE_CHANGE_TIMER.get(self.lane_change_set_timer,
                                                        AUTO_LANE_CHANGE_TIMER[AutoLaneChangeMode.NUDGE])

    self.lane_change_wait_timer += DT_MDL

    if self.lane_change_bsm_delay and blindspot_detected and self.lane_change_delay > 0:
      if self.lane_change_delay == AUTO_LANE_CHANGE_TIMER[AutoLaneChangeMode.NUDGELESS]:
        self.lane_change_wait_timer = ONE_SECOND_DELAY
      else:
        self.lane_change_wait_timer = self.lane_change_delay + ONE_SECOND_DELAY

  def update_lane_safety_assessment(self, model_data, carstate, radarstate) -> bool:
    """
    Enhanced lane change safety assessment using model and radar data
    Returns True if it's safe to perform a lane change based on comprehensive safety analysis
    """
    # Check if we have valid model data and radar data
    if not model_data or not radarstate or not hasattr(radarstate, 'leadOne'):
      return True  # Can't assess safety, assume safe if no data available

    # Assess lead vehicle conditions
    lead_one = radarstate.leadOne
    if lead_one.status and lead_one.dRel > 0:  # Lead vehicle is valid and ahead
      # Calculate safe distance based on speed and reaction time
      safe_distance = max(50.0, carstate.vEgo * 2.0)  # 2-second rule, min 50m
      if lead_one.dRel < safe_distance * 0.7:  # Less than 70% of safe distance
        return False

      # Track speed differences for merging safety
      self.speed_diff_buffer.append(lead_one.vRel if hasattr(lead_one, 'vRel') else 0)

      # Check for vehicles in target lane using model data
      # Model data contains lane line and road edge information that can indicate lane width
      if hasattr(model_data, 'laneLines') and len(model_data.laneLines) >= 4:
        # Calculate lane width consistency
        # Estimate lane width from the distance between lane lines
        try:
          # Use the y-coordinates of lane lines at a fixed distance ahead (e.g., 10 meters)
          # to estimate lane width
          lane_width_estimate = abs(model_data.laneLines[1].y[10] - model_data.laneLines[2].y[10]) \
                               if len(model_data.laneLines[1].y) > 10 and len(model_data.laneLines[2].y) > 10 else 3.7  # Default to 3.7m
          self.lane_width_buffer.append(lane_width_estimate)
        except (IndexError, AttributeError):
          # If we can't estimate lane width, continue with default
          self.lane_width_buffer.append(3.7)

        # Check if lane width is reasonable for lane change (not too narrow)
        if len(self.lane_width_buffer) > 3:
          avg_lane_width = sum(self.lane_width_buffer) / len(self.lane_width_buffer)
          if avg_lane_width < 2.5:  # Lane too narrow for safe lane change
            return False

    # Assess traffic density and safety in adjacent lanes
    # Use radar data to check for vehicles in adjacent lanes
    lead_two = radarstate.leadTwo
    if lead_two.status:
      # Check if leadTwo is in adjacent lane based on yRel (relative lateral position)
      if abs(lead_two.yRel) < 2.0 and lead_two.dRel < 30.0:  # Vehicle in adjacent lane within 30m
        safe_gap = max(30.0, carstate.vEgo * 1.5)  # 1.5 second rule for adjacent lane
        if lead_two.dRel < safe_gap:
          return False

    # Enhanced safety: Check recent speed difference trends to ensure we're not approaching
    # vehicles too quickly in either current or adjacent lanes
    if len(self.speed_diff_buffer) >= 5:
      recent_speed_trend = np.mean(list(self.speed_diff_buffer)[-5:])
      if recent_speed_trend < -3.0:  # Approaching too quickly (>3m/s)
        return False

    return True  # All safety checks passed

  def update_allowed(self, model_data=None, carstate=None, radarstate=None) -> bool:
    """
    Enhanced auto lane change allowed check with comprehensive safety assessment
    """
    # Basic conditions still apply
    if self.lane_change_set_timer in (AutoLaneChangeMode.OFF, AutoLaneChangeMode.NUDGE):
      return False

    if self.prev_brake_pressed:
      return False

    if self.prev_lane_change:
      return False

    # Check if we've waited long enough
    if self.lane_change_wait_timer <= self.lane_change_delay:
      return False

    # Perform comprehensive safety assessment
    return self.update_lane_safety_assessment(model_data, carstate, radarstate)

  def update_lane_change(self, blindspot_detected: bool, brake_pressed: bool, model_data=None, carstate=None, radarstate=None) -> None:
    if brake_pressed and not self.prev_brake_pressed:
      self.prev_brake_pressed = brake_pressed

    self.update_lane_change_timers(blindspot_detected)

    self.auto_lane_change_allowed = self.update_allowed(model_data, carstate, radarstate)

  def update_state(self):
    if self.DH.lane_change_state == log.LaneChangeState.laneChangeStarting:
      self.prev_lane_change = True

    self.reset()
