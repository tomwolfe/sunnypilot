"""
Enhanced validation system with sophisticated confidence-based decision trees
and situation-aware safety validation for Sunnypilot
"""

import numpy as np
from typing import Dict, Any, Tuple
from cereal import log
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.selfdrive.common.validation_publisher import validation_metrics_publisher


class EnhancedSafetyValidator:
    """
    Enhanced validation system with sophisticated confidence-based decision trees
    and situation-aware safety validation for Sunnypilot
    """
    
    def __init__(self):
        self.params = Params()
        self.validation_thresholds = {
            'critical': 0.6,    # Minimum for any operation
            'normal': 0.7,      # Standard operation
            'aggressive': 0.85  # More responsive behavior
        }
        
        # Situation-based factors
        self.situation_factors = {
            'highway': {
                'speed_factor': 1.1,  # Higher confidence needed at high speeds
                'traffic_factor': 1.0,
                'weather_factor': 1.0
            },
            'city': {
                'speed_factor': 0.9,  # More cautious in city
                'traffic_factor': 0.8,  # More complex traffic
                'weather_factor': 0.9   # Weather more impactful
            },
            'curvy_road': {
                'speed_factor': 0.8,   # Extra caution on curves
                'traffic_factor': 0.9,
                'weather_factor': 0.8
            }
        }
        
        # Initialize messaging for receiving model outputs
        try:
            self.sm = messaging.SubMaster(['modelV2', 'carState', 'radarState'], 
                                         ignore_alive=True)
        except Exception as e:
            cloudlog.warning(f"Could not initialize SubMaster in EnhancedSafetyValidator: {e}")
            self.sm = None
    
    def calculate_situation_aware_confidence(self, model_output: Dict[str, Any], 
                                           car_state: log.CarState, 
                                           road_condition: str = 'highway') -> Dict[str, float]:
        """
        Calculate confidence based on current driving scenario
        
        Args:
            model_output: Dictionary containing model outputs with confidence metrics
            car_state: Current car state with speed, etc.
            road_condition: Type of road ('highway', 'city', 'curvy_road')
        
        Returns:
            Dictionary with enhanced confidence metrics
        """
        # Base confidence from model outputs
        base_confidence = self._calculate_base_confidence(model_output)
        
        # Apply situation-based factors
        situation_factor = self._get_situation_factor(car_state, road_condition)
        
        # Calculate enhanced metrics
        enhanced_metrics = {
            'base_confidence': base_confidence,
            'situation_factor': situation_factor,
            'situation_adjusted_confidence': base_confidence * situation_factor,
            'speed_adjusted_confidence': self._adjust_for_speed(base_confidence, car_state.vEgo),
            'system_safe': self._is_system_safe(base_confidence, car_state, road_condition)
        }
        
        # Additional safety checks
        enhanced_metrics['lead_confidence_ok'] = self._check_lead_confidence(model_output, car_state)
        enhanced_metrics['lane_confidence_ok'] = self._check_lane_confidence(model_output)
        enhanced_metrics['overall_confidence_ok'] = enhanced_metrics['situation_adjusted_confidence'] >= 0.6
        enhanced_metrics['lane_change_safe'] = self._is_lane_change_safe(model_output, car_state, enhanced_metrics)
        
        return enhanced_metrics
    
    def _calculate_base_confidence(self, model_output: Dict[str, Any]) -> float:
        """Calculate base confidence from model outputs"""
        # Extract confidence metrics from model outputs
        lead_conf_avg = model_output.get('lead_confidence_avg', 0.0)
        lane_conf_avg = model_output.get('lane_confidence_avg', 0.0)
        road_edge_conf_avg = model_output.get('road_edge_confidence_avg', 0.0)
        temporal_consistency = model_output.get('temporal_consistency', 1.0)
        path_in_lane_validity = model_output.get('path_in_lane_validity', 0.0)
        
        # Weighted confidence calculation
        base_confidence = (
            lead_conf_avg * 0.2 +
            lane_conf_avg * 0.2 +
            road_edge_conf_avg * 0.1 +
            temporal_consistency * 0.15 +
            path_in_lane_validity * 0.15 +
            model_output.get('overall_confidence', 0.0) * 0.2
        )
        
        # Ensure confidence is within bounds
        return max(0.0, min(1.0, base_confidence))
    
    def _get_situation_factor(self, car_state: log.CarState, road_condition: str) -> float:
        """Get situation-based adjustment factor"""
        if road_condition not in self.situation_factors:
            road_condition = 'highway'  # Default
            
        factors = self.situation_factors[road_condition]
        
        # Adjust based on speed
        speed_factor = factors['speed_factor']
        if car_state.vEgo > 25:  # Above ~90 km/h
            speed_factor *= 0.9  # Extra caution at high speed
        elif car_state.vEgo < 5:  # Below ~18 km/h
            speed_factor *= 1.05  # Slightly more responsive at low speed
        
        # Combine factors
        combined_factor = (
            speed_factor * 0.4 +
            factors['traffic_factor'] * 0.3 +
            factors['weather_factor'] * 0.3
        )
        
        return min(1.1, max(0.5, combined_factor))  # Clamp between 0.5 and 1.1
    
    def _adjust_for_speed(self, base_confidence: float, v_ego: float) -> float:
        """Adjust confidence based on vehicle speed"""
        # At high speeds, require higher confidence
        if v_ego > 35:  # ~125 km/h
            return base_confidence * 0.8  # Reduce effective confidence
        elif v_ego > 25:  # ~90 km/h
            return base_confidence * 0.9
        elif v_ego < 2:  # Stationary or very slow
            return min(1.0, base_confidence * 1.1)  # Can be slightly more confident
        else:
            return base_confidence
    
    def _is_system_safe(self, base_confidence: float, car_state: log.CarState, road_condition: str) -> bool:
        """Determine if system is safe to operate"""
        # Get required confidence threshold based on situation
        if road_condition == 'city' or car_state.vEgo < 15:  # City or low speed
            required_threshold = self.validation_thresholds['normal']
        else:
            required_threshold = self.validation_thresholds['aggressive']
        
        return base_confidence >= required_threshold
    
    def _check_lead_confidence(self, model_output: Dict[str, Any], car_state: log.CarState) -> bool:
        """Check if lead vehicle detection is reliable"""
        lead_conf_avg = model_output.get('lead_confidence_avg', 0.0)
        lead_conf_max = model_output.get('lead_confidence_max', 0.0)
        
        # More stringent requirements at higher speeds
        min_lead_conf = 0.6 if car_state.vEgo > 15 else 0.5  # 0.5 at low speed, 0.6 at high speed
        
        return lead_conf_avg >= min_lead_conf and lead_conf_max >= min_lead_conf * 1.2
    
    def _check_lane_confidence(self, model_output: Dict[str, Any]) -> bool:
        """Check if lane detection is reliable"""
        lane_conf_avg = model_output.get('lane_confidence_avg', 0.0)
        lane_count = model_output.get('lane_count', 0)
        
        # Need at least 2 lanes detected with good confidence
        return lane_conf_avg >= 0.65 and lane_count >= 2
    
    def _is_lane_change_safe(self, model_output: Dict[str, Any], car_state: log.CarState, 
                           enhanced_metrics: Dict[str, float]) -> bool:
        """Determine if lane change is safe"""
        # Lane change requires higher confidence than normal operation
        if not enhanced_metrics['lane_confidence_ok']:
            return False
            
        # Check for lead vehicles in target lane
        overall_conf = model_output.get('overall_confidence', 0.0)
        lane_conf = model_output.get('lane_confidence_avg', 0.0)
        
        return overall_conf >= 0.7 and lane_conf >= 0.7  # Higher thresholds for lane changes
    
    def get_safety_recommendation(self, enhanced_metrics: Dict[str, float], 
                                car_state: log.CarState) -> Tuple[bool, str]:
        """
        Get safety recommendation based on enhanced validation metrics
        
        Returns:
            Tuple of (is_safe_to_engage, reason_string)
        """
        issues = []
        
        if not enhanced_metrics['system_safe']:
            issues.append("system confidence below threshold")
        
        if not enhanced_metrics['lead_confidence_ok']:
            issues.append("lead detection unreliable")
        
        if not enhanced_metrics['lane_confidence_ok']:
            issues.append("lane detection unreliable")
        
        if not enhanced_metrics['overall_confidence_ok']:
            issues.append("overall confidence too low")
            
        if car_state.vEgo > 5 and (car_state.leftBlinker or car_state.rightBlinker):
            # Check lane change safety if blinker is on
            if not enhanced_metrics['lane_change_safe']:
                issues.append("lane change unsafe")
        
        is_safe = len(issues) == 0
        
        if is_safe:
            reason = "All safety checks passed"
        else:
            reason = f"Safety concerns: {'; '.join(issues)}"
        
        return is_safe, reason


# Singleton instance for use across the system
enhanced_validator = EnhancedSafetyValidator()