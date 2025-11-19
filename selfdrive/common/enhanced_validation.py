"""
Simple Enhanced Validation for Sunnypilot
Essential validation functionality with minimal complexity
"""
from typing import Dict, Any, Tuple
import numpy as np


class SimpleValidation:
  """Simple validation class for essential checks"""
  
  def __init__(self):
    self.confidence_threshold = 0.6
    self.lead_threshold = 0.6
    self.lane_threshold = 0.65
    self.temporal_threshold = 0.8
  
  def validate_lead_detection(self, lead_data: Dict[str, Any]) -> Tuple[bool, float]:
    """Validate lead vehicle detection"""
    if not lead_data or 'confidence' not in lead_data:
      return False, 0.0
    
    confidence = lead_data.get('confidence', 0.0)
    is_valid = confidence >= self.lead_threshold
    return is_valid, confidence
  
  def validate_lane_detection(self, left_lane_confidence: float, 
                             right_lane_confidence: float) -> Tuple[bool, float]:
    """Validate lane detection"""
    avg_confidence = (left_lane_confidence + right_lane_confidence) / 2
    is_valid = avg_confidence >= self.lane_threshold
    return is_valid, avg_confidence
  
  def validate_temporal_consistency(self, current_value: float, 
                                   historical_values: list, threshold: float = 0.1) -> bool:
    """Validate temporal consistency"""
    if not historical_values:
      return True
    
    avg_historical = sum(historical_values) / len(historical_values)
    return abs(current_value - avg_historical) <= threshold
  
  def get_overall_safety_score(self, lead_valid: bool, lane_valid: bool, 
                              temporal_valid: bool, lead_conf: float = 0.0,
                              lane_conf: float = 0.0) -> Dict[str, Any]:
    """Get overall safety validation score"""
    score = 0.0
    
    if lead_valid:
      score += lead_conf * 0.4  # Lead detection is important
    if lane_valid:
      score += lane_conf * 0.4  # Lane detection is important
    if temporal_valid:
      score += 0.2  # Consistency adds to safety
    
    is_safe = score >= self.confidence_threshold
    
    return {
      'overallConfidence': score,
      'isValid': is_safe,
      'systemShouldEngage': is_safe,
      'safetyScore': score
    }