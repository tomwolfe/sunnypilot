"""
Traffic Light Temporal Filter

Implements temporal consistency filtering for traffic light states to reduce
flickering between states and improve reliability of traffic light detection.
"""
import numpy as np
from collections import deque
from enum import IntEnum


class TrafficLightState(IntEnum):
  """Traffic light states"""
  RED = 0
  YELLOW = 1
  GREEN = 2
  UNKNOWN = 3


class TrafficLightTemporalFilter:
  """Temporal filter for traffic light states with hysteresis and confidence tracking"""
  
  def __init__(self, buffer_size=5, hysteresis_factor=0.2, min_confidence_for_change=0.5):
    """
    Initialize the traffic light temporal filter

    Args:
      buffer_size: Size of the history buffer for temporal smoothing
      hysteresis_factor: Factor to prevent rapid state changes (0.0-1.0)
      min_confidence_for_change: Minimum confidence required to change from previous state
    """
    self.buffer_size = buffer_size
    self.hysteresis_factor = hysteresis_factor
    self.min_confidence_for_change = min_confidence_for_change  # Explicit parameter for clarity
    self.state_buffer = deque(maxlen=buffer_size)
    self.confidence_buffer = deque(maxlen=buffer_size)

    # Store previous state to apply hysteresis
    self.prev_state = TrafficLightState.UNKNOWN
    self.prev_confidence = 0.0

    # Counter to track consecutive UNKNOWN state detections for more stable transitions
    self.unknown_counter = 0
    self.min_unknown_samples = 3  # Minimum number of samples before allowing state transition from UNKNOWN
    
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
    # Determine initial state based on thresholds
    if raw_score < green_threshold:
      current_state = TrafficLightState.GREEN
      initial_confidence = 1.0 - (raw_score / green_threshold)  # Higher confidence when score is much lower than threshold
    elif raw_score < yellow_threshold:
      current_state = TrafficLightState.YELLOW
      # Confidence based on how close score is to green vs yellow thresholds
      conf_to_green = (raw_score - green_threshold) / (yellow_threshold - green_threshold)
      initial_confidence = 1.0 - conf_to_green
    else:
      current_state = TrafficLightState.RED
      # Improved confidence calculation for RED state:
      # Use the range from yellow_threshold to a reasonable maximum possible score (e.g., 1.0)
      # This makes the calculation more principled rather than using an arbitrary 0.5 multiplier
      max_possible_score = 1.0  # Assumption: model scores are in range [0, 1]
      confidence_range = max_possible_score - yellow_threshold
      if confidence_range > 0:
        initial_confidence = min(1.0, (raw_score - yellow_threshold) / confidence_range)
      else:
        initial_confidence = 0.0  # Safety fallback
    
    # Apply temporal smoothing to confidence
    # The confidence buffer is used separately from state buffer to smooth confidence values over time
    self.confidence_buffer.append(initial_confidence)
    if len(self.confidence_buffer) > 0:
      # Use weighted average to smooth confidence over time
      smoothed_confidence = np.mean(list(self.confidence_buffer))
    else:
      smoothed_confidence = initial_confidence

    # Apply hysteresis to prevent rapid state changes
    # Only change state if we have sufficient confidence AND different from previous state
    if current_state != self.prev_state:
      # Special handling for transitions from UNKNOWN state
      if self.prev_state == TrafficLightState.UNKNOWN:
        # When in UNKNOWN state, require multiple consecutive detections before transitioning
        if current_state != TrafficLightState.UNKNOWN:
          self.unknown_counter += 1
          if self.unknown_counter >= self.min_unknown_samples:
            # Allow transition after sufficient consecutive detections
            final_state = current_state
            self.prev_state = current_state
            self.prev_confidence = smoothed_confidence
            self.unknown_counter = 0  # Reset counter after successful transition
          else:
            # Stay in UNKNOWN until we have enough consistent detections
            final_state = self.prev_state
            smoothed_confidence = max(smoothed_confidence, self.prev_confidence * 0.8)  # Reduced confidence
        else:
          # Staying in UNKNOWN state, reset counter
          final_state = current_state
          self.prev_confidence = smoothed_confidence
          self.unknown_counter = 0
      else:
        # Regular hysteresis for non-UNKNOWN transitions
        # Require minimum confidence to change from previous state
        # Use the explicit min_confidence_for_change parameter for clarity
        if smoothed_confidence > self.min_confidence_for_change:
          final_state = current_state
          self.prev_state = current_state
          self.prev_confidence = smoothed_confidence
        else:
          # Don't change state, keep previous
          final_state = self.prev_state
          smoothed_confidence = max(smoothed_confidence, self.prev_confidence * 0.8)  # Reduced confidence
    else:
      # Same state as previous, reset unknown counter if we were in UNKNOWN
      if self.prev_state == TrafficLightState.UNKNOWN and current_state != TrafficLightState.UNKNOWN:
        self.unknown_counter = 0  # Reset counter since we're no longer in UNKNOWN

      final_state = current_state
      self.prev_confidence = smoothed_confidence

    # Add to state buffer for temporal consistency
    # The state buffer is separate from the confidence buffer and is used for majority voting
    # to add additional stability to state decisions while confidence continues to be smoothed independently
    self.state_buffer.append(final_state)

    # Apply majority voting for additional stability (optional)
    if len(self.state_buffer) >= 3:
      unique_states, counts = np.unique(list(self.state_buffer), return_counts=True)
      most_common_idx = np.argmax(counts)
      majority_state = TrafficLightState(unique_states[most_common_idx])

      # Only use majority vote if it matches current state or confidence is low
      if (majority_state == final_state or smoothed_confidence < 0.6):
        final_state = majority_state
    
    return final_state, smoothed_confidence


# Global instance for use in the model pipeline
_traffic_light_filter = TrafficLightTemporalFilter()


def get_traffic_light_filter():
  """Get the global traffic light filter instance"""
  return _traffic_light_filter