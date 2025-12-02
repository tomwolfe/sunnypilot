"""
Traffic Light Temporal Filter

Implements temporal consistency filtering for traffic light states to reduce
flickering between states and improve reliability of traffic light detection.

NOTE ON CONFIDENCE CALCULATION:
The confidence calculation varies by state to handle different semantic meanings of the raw score:
- For GREEN state: confidence increases as raw_score gets lower relative to green_threshold
- For YELLOW state: confidence is based on proximity to the center of the green-yellow range
- For RED state: confidence increases as raw_score gets higher above yellow_threshold

This approach is necessary because different traffic light colors produce different
raw score patterns in the model output, so a unified approach would not be appropriate.
"""
from collections import deque
from enum import IntEnum


class TrafficLightState(IntEnum):
  """Traffic light states"""
  RED = 0
  YELLOW = 1
  GREEN = 2
  UNKNOWN = 3


# Configurable constants
DEFAULT_BUFFER_SIZE = 5
HYSTERESIS_FACTOR = 0.2
MIN_CONFIDENCE_FOR_CHANGE = 0.5
MIN_UNKNOWN_SAMPLES = 3  # Minimum number of samples before allowing state transition from UNKNOWN
UNKNOWN_COUNTER_RESET_VALUE = 0
DEFAULT_MAX_POSSIBLE_SCORE = 1.0  # Default assumption for model score range


class TrafficLightTemporalFilter:
  """Temporal filter for traffic light states with hysteresis and confidence tracking"""

  def __init__(self, buffer_size=DEFAULT_BUFFER_SIZE, hysteresis_factor=HYSTERESIS_FACTOR,
               min_confidence_for_change=MIN_CONFIDENCE_FOR_CHANGE, max_possible_score=DEFAULT_MAX_POSSIBLE_SCORE):
    """
    Initialize the traffic light temporal filter

    Args:
      buffer_size: Size of the history buffer for temporal smoothing
      hysteresis_factor: Factor to prevent rapid state changes (0.0-1.0)
      min_confidence_for_change: Minimum confidence required to change from previous state
      max_possible_score: Maximum possible score from the model (for confidence calculation)
    """
    self.buffer_size = buffer_size
    self.hysteresis_factor = hysteresis_factor
    self.min_confidence_for_change = min_confidence_for_change  # Explicit parameter for clarity
    self.max_possible_score = max_possible_score  # Parameter for robust confidence calculation
    self.state_buffer = deque(maxlen=buffer_size)
    self.confidence_buffer = deque(maxlen=buffer_size)

    # For efficient running average calculation
    self.running_confidence_sum = 0.0

    # Store previous state to apply hysteresis
    self.prev_state = TrafficLightState.UNKNOWN
    self.prev_confidence = 0.0

    # Counter to track consecutive UNKNOWN state detections for more stable transitions
    self.unknown_counter = 0
    self.min_unknown_samples = MIN_UNKNOWN_SAMPLES  # Minimum number of samples before allowing state transition from UNKNOWN

    # Dynamic max score estimation
    self.score_history = deque(maxlen=50)  # Keep track of recent scores to estimate max dynamically

  def update(self, raw_score: float, green_threshold: float, yellow_threshold: float) -> tuple[TrafficLightState, float]:
    """
    Update the filter with a new traffic light detection score

    Args:
      raw_score: Raw model output score for traffic light confidence
      green_threshold: Threshold for green light classification
      yellow_threshold: Threshold for yellow light classification

    Returns:
      tuple of (filtered_state, confidence)
    """
    # Add current score to history for dynamic max estimation
    self.score_history.append(raw_score)

    # Update max_possible_score based on recent history if we see higher scores
    if len(self.score_history) > 10:  # Only adjust if we have sufficient history
      estimated_max = max(self.score_history)
      # Gradually update max_possible_score to avoid sudden changes
      self.max_possible_score = max(self.max_possible_score, estimated_max * 1.1)  # Allow 10% buffer above observed max

    # Determine initial state based on thresholds
    # NOTE: The confidence calculation varies by state to handle different semantic meanings of the raw score
    # For GREEN: low raw_score means high confidence in GREEN state
    # For YELLOW: raw_score between green and yellow thresholds means YELLOW state
    # For RED: high raw_score above yellow threshold means high confidence in RED state
    if raw_score < green_threshold:
      current_state = TrafficLightState.GREEN
      # For GREEN state: confidence increases as score gets lower relative to green_threshold
      # Higher confidence when score is much lower than threshold
      initial_confidence = 1.0 - (raw_score / green_threshold) if green_threshold > 0 else 1.0
    elif raw_score < yellow_threshold:
      current_state = TrafficLightState.YELLOW
      # For YELLOW state: confidence is highest when the score is between green and yellow thresholds
      # Calculate confidence based on position within the yellow range
      # 1.0 at center, decreasing toward edges
      yellow_range_center = (green_threshold + yellow_threshold) / 2.0
      distance_from_center = abs(raw_score - yellow_range_center)
      max_distance_from_center = (yellow_threshold - green_threshold) / 2.0
      if max_distance_from_center > 0:
        # Normalize distance from center (0 at center, 1 at edges)
        normalized_distance = distance_from_center / max_distance_from_center
        # Confidence is 1.0 at center, 0.0 at edges
        initial_confidence = max(0.0, 1.0 - normalized_distance)
      else:
        initial_confidence = 1.0
    else:
      current_state = TrafficLightState.RED
      # For RED state: confidence increases as score gets higher above yellow_threshold
      # Use the range from yellow_threshold to a reasonable maximum possible score
      confidence_range = self.max_possible_score - yellow_threshold
      if confidence_range > 0:
        initial_confidence = min(1.0, max(0.0, (raw_score - yellow_threshold) / confidence_range))
      else:
        initial_confidence = 0.0  # Safety fallback

    # Apply temporal smoothing to confidence
    # The confidence buffer is used separately from state buffer to smooth confidence values over time
    # Use running average for efficiency instead of converting deque to list each time
    if len(self.confidence_buffer) >= self.buffer_size:
      # Remove the oldest value from the running sum
      oldest_value = self.confidence_buffer[0]
      self.running_confidence_sum -= oldest_value

    # Add the new value to the buffer and running sum
    self.confidence_buffer.append(initial_confidence)
    self.running_confidence_sum += initial_confidence

    # Calculate smoothed confidence as running average
    if len(self.confidence_buffer) > 0:
      smoothed_confidence = self.running_confidence_sum / len(self.confidence_buffer)
    else:
      smoothed_confidence = initial_confidence

    # Apply hysteresis to prevent rapid state changes
    # Use hysteresis_factor to adjust the threshold based on the previous state
    # Different states may require different confidence levels to transition from
    adjusted_confidence_threshold = self.min_confidence_for_change

    # Apply hysteresis based on the previous state and the hysteresis factor
    if self.prev_state != TrafficLightState.UNKNOWN:
      # For transitions from known states, apply hysteresis factor to prevent rapid changes
      # Make it harder to change from the current state than to maintain it
      if current_state != self.prev_state:
        # When transitioning from a known state to a different state, increase the required confidence
        # Use hysteresis_factor to scale the difference between current and previous confidence
        required_difference = self.hysteresis_factor * (1.0 - self.prev_confidence)
        adjusted_confidence_threshold = max(self.min_confidence_for_change, self.prev_confidence + required_difference)

    # Only change state if we have sufficient confidence AND different from previous state
    if current_state != self.prev_state:
      # Special handling for transitions from UNKNOWN state
      if self.prev_state == TrafficLightState.UNKNOWN:
        # When in UNKNOWN state, require multiple consecutive detections before transitioning
        if current_state != TrafficLightState.UNKNOWN:
          self.unknown_counter += 1
          if self.unknown_counter >= self.min_unknown_samples:
            # Allow transition after sufficient consecutive detections
            # Only if confidence meets the adjusted threshold
            if smoothed_confidence > adjusted_confidence_threshold:
              final_state = current_state
              self.prev_state = current_state
              self.prev_confidence = smoothed_confidence
              self.unknown_counter = UNKNOWN_COUNTER_RESET_VALUE  # Reset counter after successful transition
            else:
              # Don't change state if confidence is too low
              final_state = self.prev_state
              # Use gradual decay instead of fixed 0.8 multiplier
              confidence_decay_factor = 0.9  # Gradual decay to maintain confidence when "stuck"
              smoothed_confidence = max(smoothed_confidence, self.prev_confidence * confidence_decay_factor)
          else:
            # Stay in UNKNOWN until we have enough consistent detections
            final_state = self.prev_state
            # Use gradual decay instead of fixed 0.8 multiplier
            confidence_decay_factor = 0.9  # Gradual decay to maintain confidence when "stuck"
            smoothed_confidence = max(smoothed_confidence, self.prev_confidence * confidence_decay_factor)
        else:
          # Staying in UNKNOWN state, reset counter
          final_state = current_state
          self.prev_confidence = smoothed_confidence
          self.unknown_counter = UNKNOWN_COUNTER_RESET_VALUE
      else:
        # Regular hysteresis for non-UNKNOWN transitions
        # Require minimum confidence to change from previous state
        # Use the dynamically adjusted confidence threshold
        if smoothed_confidence > adjusted_confidence_threshold:
          final_state = current_state
          self.prev_state = current_state
          self.prev_confidence = smoothed_confidence
        else:
          # Don't change state, keep previous
          final_state = self.prev_state
          # Maintain previous confidence to avoid state flickering when confidence is insufficient to transition
          # Apply a gradual decay to prevent permanent memory of high confidence states
          confidence_decay_factor = 0.9  # Gradual decay to maintain confidence when "stuck"
          smoothed_confidence = max(smoothed_confidence, self.prev_confidence * confidence_decay_factor)
    else:
      # Same state as previous, reset unknown counter if we were in UNKNOWN
      if self.prev_state == TrafficLightState.UNKNOWN and current_state != TrafficLightState.UNKNOWN:
        self.unknown_counter = UNKNOWN_COUNTER_RESET_VALUE  # Reset counter since we're no longer in UNKNOWN

      final_state = current_state
      self.prev_confidence = smoothed_confidence

    # Add to state buffer for temporal consistency
    # The state buffer and confidence buffer work together: the confidence buffer provides
    # smoothed confidence values used in hysteresis decisions, while the state buffer provides
    # historical state information used for majority voting to add additional stability
    self.state_buffer.append(final_state)

    # Apply majority voting for additional stability (optional)
    MAJORITY_VOTE_THRESHOLD = 3  # Minimum number of states before applying majority voting
    MAJORITY_CONFIDENCE_THRESHOLD = 0.6  # Confidence threshold for majority voting consideration

    if len(self.state_buffer) >= MAJORITY_VOTE_THRESHOLD:
      # Count states in the buffer more efficiently
      state_counts: dict[TrafficLightState, int] = {}
      for state in self.state_buffer:
        state_counts[state] = state_counts.get(state, 0) + 1

      # Find the most common state
      most_common_state = max(state_counts, key=lambda k: state_counts[k])
      majority_state = TrafficLightState(most_common_state)

      # Calculate the proportion of majority votes
      majority_proportion = state_counts[most_common_state] / len(self.state_buffer)

      # Only use majority vote if it matches current state or confidence is low
      # Also consider the proportion of majority votes to prevent flipping due to small majorities
      if ((majority_state == final_state or smoothed_confidence < MAJORITY_CONFIDENCE_THRESHOLD)
          and majority_proportion > 0.5):  # Ensure true majority (more than half)
        final_state = majority_state

    return final_state, smoothed_confidence


# Global instance for use in the model pipeline
_traffic_light_filter = TrafficLightTemporalFilter(buffer_size=DEFAULT_BUFFER_SIZE,
                                                  hysteresis_factor=HYSTERESIS_FACTOR,
                                                  min_confidence_for_change=MIN_CONFIDENCE_FOR_CHANGE,
                                                  max_possible_score=DEFAULT_MAX_POSSIBLE_SCORE)


def get_traffic_light_filter():
  """Get the global traffic light filter instance"""
  return _traffic_light_filter
