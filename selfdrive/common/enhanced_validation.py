"""
Enhanced validation system for Sunnypilot with situation-aware confidence scoring
"""
from typing import Dict, Any, Optional
from cereal import log, custom
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
import numpy as np
import time


class EnhancedValidation:
  def __init__(self):
    self.params = Params()
    self.last_validation_time = 0
    self.validation_interval = 0.05  # 20Hz validation
    self.situation_factors = {
      'weather': 1.0,
      'lighting': 1.0,
      'traffic_density': 1.0,
      'road_type': 1.0
    }

    # Initialize messaging
    try:
      self.sm = messaging.SubMaster([
        'modelV2', 'carState', 'radarState', 'deviceState',
        'liveCalibration', 'driverMonitoring', 'roadCameraState'
      ])
    except Exception as e:
      cloudlog.error(f"Could not initialize SubMaster in EnhancedValidation: {e}")
      self.sm = None

  def run_validation(self, sm: Optional[messaging.SubMaster] = None) -> Dict[str, Any]:
    """
    Run comprehensive validation with situation-aware confidence scoring
    """
    current_time = time.time()

    # Limit validation frequency
    if current_time - self.last_validation_time < self.validation_interval:
      return {
        "systemSafe": True,
        "reasons": ["Validation rate limiting"],
        "confidence": 0.8,
        "situationFactor": 1.0
      }

    self.last_validation_time = current_time

    # Use provided SubMaster or fall back to instance SubMaster
    validation_sm = sm or self.sm

    if not validation_sm:
      return {
        "systemSafe": False,
        "reasons": ["Messaging not initialized"],
        "confidence": 0.0,
        "situationFactor": 1.0
      }

    try:
      # Get model outputs for validation
      model_valid = validation_sm.valid['modelV2'] if 'modelV2' in validation_sm.valid else False
      model_output = validation_sm['modelV2'] if 'modelV2' in validation_sm else None

      car_state = validation_sm['carState'] if 'carState' in validation_sm else None
      radar_state = validation_sm['radarState'] if 'radarState' in validation_sm else None

      if not model_valid or model_output is None:
        return {
          "systemSafe": False,
          "reasons": ["Model output invalid"],
          "confidence": 0.0,
          "situationFactor": 1.0
        }

      # Calculate base validation metrics
      base_metrics = self._calculate_base_metrics(model_output, car_state)

      # Calculate situation-aware factors
      situation_factor = self._calculate_situation_factor(validation_sm)

      # Apply situation factor to base metrics
      adjusted_metrics = self._apply_situation_adjustments(base_metrics, situation_factor)

      # Determine overall system safety
      system_safe = self._determine_system_safety(adjusted_metrics, car_state, radar_state)

      # Calculate final confidence
      confidence = self._calculate_overall_confidence(adjusted_metrics, situation_factor)

      result = {
        "systemSafe": system_safe,
        "reasons": adjusted_metrics.get('violations', []),
        "confidence": confidence,
        "situationFactor": situation_factor,
        "base_metrics": base_metrics,
        "adjusted_metrics": adjusted_metrics
      }

      return result

    except Exception as e:
      cloudlog.error(f"Error in enhanced validation: {e}")
      return {
        "systemSafe": False,
        "reasons": [f"Validation error: {str(e)}"],
        "confidence": 0.0,
        "situationFactor": 1.0
      }

  def _calculate_base_metrics(self, model_output, car_state) -> Dict[str, Any]:
    """Calculate base validation metrics from model output"""
    violations = []

    # Calculate lead detection confidence
    lead_confidences = []
    if hasattr(model_output, 'leadsV3') and model_output.leadsV3:
      for lead in model_output.leadsV3:
        if hasattr(lead, 'prob'):
          lead_confidences.append(lead.prob)

    lead_conf_avg = np.mean(lead_confidences) if lead_confidences else 0.0
    lead_conf_max = max(lead_confidences) if lead_confidences else 0.0

    # Calculate lane detection confidence
    lane_confidences = []
    if hasattr(model_output, 'laneLines') and model_output.laneLines:
      for line in model_output.laneLines:
        if hasattr(line, 'prob'):
          lane_confidences.append(line.prob)

    lane_conf_avg = np.mean(lane_confidences) if lane_confidences else 0.0

    # Check for validation thresholds
    if lead_conf_avg < 0.5:
      violations.append(f"Lead confidence too low: {lead_conf_avg:.2f}")
    if lane_conf_avg < 0.65:
      violations.append(f"Lane confidence too low: {lane_conf_avg:.2f}")

    # Check path validity
    path_in_lane_valid = 0.0
    if hasattr(model_output, 'position') and hasattr(model_output.position, 'y'):
      # Check if planned path stays within lane boundaries
      path_deviation = np.max(np.abs(model_output.position.y))
      path_in_lane_valid = max(0.0, 1.0 - (path_deviation / 1.5))  # Normalize against 1.5m threshold

    return {
      'leadConfidenceAvg': lead_conf_avg,
      'leadConfidenceMax': lead_conf_max,
      'laneConfidenceAvg': lane_conf_avg,
      'roadEdgeConfidenceAvg': 0.0,  # Would need roadEdges from modelV2
      'overallConfidence': (lead_conf_avg + lane_conf_avg) / 2,
      'pathInLaneValid': path_in_lane_valid,
      'violations': violations
    }

  def _calculate_situation_factor(self, sm) -> float:
    """Calculate situation factor based on environmental conditions"""
    situation_factor = 1.0

    # Consider weather (simulated via parameters or external data)
    try:
      weather_factor = float(self.params.get("WeatherFactor", "1.0"))
      situation_factor *= weather_factor
    except:
      situation_factor *= 1.0

    # Consider vehicle speed - reduce factor at higher speeds in poor conditions
    if 'carState' in sm and sm['carState'] is not None:
      v_ego = sm['carState'].vEgo if hasattr(sm['carState'], 'vEgo') else 0.0
      if v_ego > 25.0 and situation_factor < 0.8:  # Above ~90 km/h in poor conditions
        situation_factor *= 0.9

    # Consider time of day (would come from external source)
    try:
      time_of_day_factor = float(self.params.get("TimeOfDayFactor", "1.0"))
      situation_factor *= time_of_day_factor
    except:
      situation_factor *= 1.0

    # Clamp to reasonable range
    return max(0.1, min(1.0, situation_factor))

  def _apply_situation_adjustments(self, base_metrics: Dict[str, Any], situation_factor: float) -> Dict[str, Any]:
    """Apply situation factor to base metrics"""
    adjusted = base_metrics.copy()

    # Adjust confidences based on situation
    adjusted['leadConfidenceAvg'] *= situation_factor
    adjusted['leadConfidenceMax'] *= situation_factor
    adjusted['laneConfidenceAvg'] *= situation_factor
    adjusted['overallConfidence'] *= situation_factor

    # Update violations if adjustments cause new ones
    new_violations = base_metrics.get('violations', []).copy()
    if adjusted['leadConfidenceAvg'] < 0.5:
      new_violations.append(f"Situation-adjusted lead confidence too low: {adjusted['leadConfidenceAvg']:.2f}")
    if adjusted['laneConfidenceAvg'] < 0.65:
      new_violations.append(f"Situation-adjusted lane confidence too low: {adjusted['laneConfidenceAvg']:.2f}")

    adjusted['violations'] = new_violations
    return adjusted

  def _determine_system_safety(self, adjusted_metrics: Dict[str, Any], car_state, radar_state) -> bool:
    """Determine if system is safe to operate based on adjusted metrics"""
    # Check if we have valid car state
    if car_state is None:
      return False

    # Check if any violations exist
    if adjusted_metrics.get('violations'):
      return False

    # Check confidence thresholds
    lead_conf = adjusted_metrics.get('leadConfidenceAvg', 0.0)
    lane_conf = adjusted_metrics.get('laneConfidenceAvg', 0.0)
    overall_conf = adjusted_metrics.get('overallConfidence', 0.0)

    # Apply safety thresholds
    if lead_conf < 0.6 or lane_conf < 0.65 or overall_conf < 0.6:
      return False

    # Check for radar-based safety (if available)
    if radar_state and hasattr(radar_state, 'leadOne') and radar_state.leadOne.status:
      # Check if we're approaching a lead too quickly
      if (hasattr(radar_state.leadOne, 'dRel') and
          hasattr(radar_state.leadOne, 'vRel') and
          radar_state.leadOne.dRel < 50.0 and  # Within 50m
          radar_state.leadOne.vRel < -5.0):    # Approaching faster than 5 m/s
        return False

    # Check vehicle state safety
    v_ego = car_state.vEgo if hasattr(car_state, 'vEgo') else 0.0
    if v_ego < 0.5 and adjusted_metrics.get('pathInLaneValid', 0.0) < 0.7:
      # At low speed, path validity is critical for safety
      return False

    return True

  def _calculate_overall_confidence(self, adjusted_metrics: Dict[str, Any], situation_factor: float) -> float:
    """Calculate overall system confidence"""
    base_conf = adjusted_metrics.get('overallConfidence', 0.0)

    # Combine base confidence with situation awareness
    confidence = (base_conf * 0.7 + situation_factor * 0.3)  # Weighted combination
    return max(0.0, min(1.0, confidence))


# Global instance for use across the system
enhanced_validator = EnhancedValidation()
