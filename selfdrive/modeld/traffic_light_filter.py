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
  
  def __init__(self, buffer_size=5, hysteresis_factor=0.2):
    """
    Initialize the traffic light temporal filter
    
    Args:
      buffer_size: Size of the history buffer for temporal smoothing
      hysteresis_factor: Factor to prevent rapid state changes (0.0-1.0)
    """
    self.buffer_size = buffer_size
    self.hysteresis_factor = hysteresis_factor
    self.state_buffer = deque(maxlen=buffer_size)
    self.confidence_buffer = deque(maxlen=buffer_size)
    
    # Store previous state to apply hysteresis
    self.prev_state = TrafficLightState.UNKNOWN
    self.prev_confidence = 0.0
    
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
      initial_confidence = min(1.0, (raw_score - yellow_threshold) / (yellow_threshold * 0.5))  # Confidence increases with score
    
    # Apply temporal smoothing to confidence
    self.confidence_buffer.append(initial_confidence)
    if len(self.confidence_buffer) > 0:
      # Use weighted average to smooth confidence over time
      smoothed_confidence = np.mean(list(self.confidence_buffer))
    else:
      smoothed_confidence = initial_confidence
    
    # Apply hysteresis to prevent rapid state changes
    # Only change state if we have sufficient confidence AND different from previous state
    if current_state != self.prev_state:
      # Require higher confidence to change from previous state
      hysteresis_threshold = self.hysteresis_factor + 0.3  # Higher threshold for state changes
      if smoothed_confidence > hysteresis_threshold:
        final_state = current_state
        self.prev_state = current_state
        self.prev_confidence = smoothed_confidence
      else:
        # Don't change state, keep previous
        final_state = self.prev_state
        smoothed_confidence = max(smoothed_confidence, self.prev_confidence * 0.8)  # Reduced confidence
    else:
      final_state = current_state
      self.prev_confidence = smoothed_confidence
    
    # Add to state buffer for temporal consistency
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