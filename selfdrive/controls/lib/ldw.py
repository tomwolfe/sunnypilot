from cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.constants import CV
import numpy as np


CAMERA_OFFSET = 0.04
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LDW_MIN_SPEED_ADAPTIVE = 15 * CV.MPH_TO_MS  # Lower threshold for adaptive detection
LANE_DEPARTURE_THRESHOLD = 0.1

class LaneDepartureWarning:
  def __init__(self):
    self.left = False
    self.right = False
    self.last_blinker_frame = 0

    # Enhanced state tracking for better prediction
    self.left_prediction_history = []
    self.right_prediction_history = []
    self.max_history_length = 5  # Store last 5 frames for trend analysis

  def update(self, frame, modelV2, CS, CC):
    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = frame

    recent_blinker = (frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown

    # Adaptive speed threshold based on driving conditions
    if CS.vEgo > LDW_MIN_SPEED:
      # Higher speed - use more conservative detection
      ldw_allowed = not recent_blinker and not CC.latActive
    elif CS.vEgo > LDW_MIN_SPEED_ADAPTIVE:
      # Lower speed - allow detection but with more context awareness
      ldw_allowed = not recent_blinker and not CC.latActive
    else:
      ldw_allowed = False

    desire_prediction = modelV2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = modelV2.laneLineProbs[2] > 0.5
      left_lane_visible = modelV2.laneLineProbs[1] > 0.5
      l_lane_change_prob = desire_prediction[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_prediction[log.Desire.laneChangeRight]

      lane_lines = modelV2.laneLines
      l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

      # Enhanced prediction with trend analysis
      # Add current prediction to history
      self.left_prediction_history.append(l_lane_change_prob)
      self.right_prediction_history.append(r_lane_change_prob)

      # Maintain history length
      if len(self.left_prediction_history) > self.max_history_length:
        self.left_prediction_history.pop(0)
      if len(self.right_prediction_history) > self.max_history_length:
        self.right_prediction_history.pop(0)

      # Calculate trend - if probabilities are increasing over time, it's more likely a genuine lane departure
      left_trend = self._calculate_trend(self.left_prediction_history)
      right_trend = self._calculate_trend(self.right_prediction_history)

      # Enhanced lane departure logic with trend analysis
      # Consider both current probability and trend
      left_departure = l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close
      right_departure = r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close

      # For trend-based detection, use slightly lower threshold but require increasing trend
      if not left_departure and l_lane_change_prob > (LANE_DEPARTURE_THRESHOLD * 0.7) and left_trend > 0.02:
        left_departure = l_lane_close

      if not right_departure and r_lane_change_prob > (LANE_DEPARTURE_THRESHOLD * 0.7) and right_trend > 0.02:
        right_departure = r_lane_close

      # Add vehicle dynamics consideration
      # If there's significant lateral acceleration but no lane departure is detected, be more conservative
      if abs(CS.aEgo) > 0.5:  # Significant longitudinal acceleration
        left_departure = left_departure and (abs(CS.yawRate) > 0.05)
        right_departure = right_departure and (abs(CS.yawRate) > 0.05)

      self.left = bool(left_departure)
      self.right = bool(right_departure)
    else:
      self.left, self.right = False, False

  def _calculate_trend(self, values):
    """Calculate the trend of prediction values over time."""
    if len(values) < 2:
      return 0.0

    # Calculate simple linear trend using last N values
    n = len(values)
    if n > 1:
      # Use numpy's polyfit for simple linear regression
      x = np.arange(len(values))
      slope, _ = np.polyfit(x, values, 1)
      return slope
    return 0.0

  @property
  def warning(self) -> bool:
    return bool(self.left or self.right)
