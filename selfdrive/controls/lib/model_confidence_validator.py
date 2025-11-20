#!/usr/bin/env python3
"""
Model confidence validation module for sunnypilot
Provides enhanced confidence validation and cross-verification systems
"""

import numpy as np
from typing import Dict, Any, Tuple, Optional
from openpilot.common.swaglog import cloudlog
from cereal import log
from .autonomous_params import LATERAL_SAFETY_PARAMS, SAFETY_PARAMETERS
from .safety_state_manager import ERROR_HANDLING


class ModelConfidenceValidator:
    """
    Enhanced model confidence validation with cross-verification
    """

    def __init__(self):
        self.confidence_history = []
        self.max_history = 100  # Keep last 100 confidence readings
        # Use centralized threshold from autonomous_params to ensure consistency
        self.confidence_threshold = SAFETY_PARAMETERS['MODEL_CONFIDENCE_THRESHOLD']  # Minimum confidence for safe operation
        self.low_confidence_count = 0
        self.max_low_confidence = ERROR_HANDLING['max_consecutive_failures']  # From standardized error handling policy
        
    def validate_model_output(self, model_output: Dict[str, Any], 
                            v_ego: float, 
                            curvature: float) -> Tuple[float, bool, Dict[str, Any]]:
        """
        Validate model output and return enhanced confidence score
        :param model_output: Raw model output to validate
        :param v_ego: Current vehicle speed
        :param curvature: Current road curvature
        :return: Tuple of (enhanced_confidence, is_safe, validation_details)
        """
        validation_details = {}
        
        # Get base confidence from model
        base_confidence = self._get_base_confidence(model_output)
        validation_details['base_confidence'] = base_confidence
        
        # Perform path consistency check
        path_consistency = self._check_path_consistency(model_output)
        validation_details['path_consistency'] = path_consistency
        
        # Perform curvature smoothness check
        curvature_smoothness = self._check_curvature_smoothness(model_output, v_ego, curvature)
        validation_details['curvature_smoothness'] = curvature_smoothness
        
        # Perform temporal consistency check
        temporal_consistency = self._check_temporal_consistency(model_output)
        validation_details['temporal_consistency'] = temporal_consistency
        
        # Calculate enhanced confidence based on all validation factors
        enhanced_confidence = self._calculate_enhanced_confidence(
            base_confidence, path_consistency, curvature_smoothness, temporal_consistency, v_ego
        )
        
        # Track confidence history
        self.confidence_history.append(enhanced_confidence)
        if len(self.confidence_history) > self.max_history:
            self.confidence_history.pop(0)
        
        # Check if confidence is low for consecutive readings
        if enhanced_confidence < self.confidence_threshold:
            self.low_confidence_count += 1
        else:
            self.low_confidence_count = 0

        # Implement standardized error handling policy
        is_safe = True
        if self.low_confidence_count >= ERROR_HANDLING['max_consecutive_failures']:
            is_safe = False  # Disengage after max failures
        elif self.low_confidence_count >= 6:
            # Apply 90% conservative factor after 6 failures
            enhanced_confidence *= 0.9
        elif self.low_confidence_count >= 3:
            # Apply 70% conservative factor after 3 failures
            enhanced_confidence *= 0.7

        validation_details['low_confidence_count'] = self.low_confidence_count

        return enhanced_confidence, is_safe, validation_details
    
    def _get_base_confidence(self, model_output: Dict[str, Any]) -> float:
        """Extract base confidence from model output"""
        # Default to high confidence if confidence info not available
        base_confidence = 1.0
        
        if 'modelConfidence' in model_output:
            base_confidence = model_output['modelConfidence']
        elif 'confidence' in model_output:
            base_confidence = model_output['confidence']
        elif 'pathStd' in model_output:  # Lower std = higher confidence
            # Convert standard deviation to confidence (inverse relationship)
            path_std = model_output['pathStd']
            base_confidence = max(0.0, min(1.0, 1.0 - path_std))
        
        return float(base_confidence)
    
    def _check_path_consistency(self, model_output: Dict[str, Any]) -> float:
        """Check consistency of path prediction"""
        try:
            # Check if lane lines are consistent with predicted path
            if 'laneLine' in model_output and 'path' in model_output:
                lane_lines = model_output['laneLine']
                path = model_output['path']
                
                # Calculate variance in path prediction vs lane alignment
                consistency_score = 1.0
                
                # If lane lines are very far from path, confidence should be lower
                if hasattr(lane_lines, '__iter__') and hasattr(path, '__iter__'):
                    try:
                        # Simple check: how well do path and lane lines align?
                        path_variance = np.var(path) if len(path) > 1 else 0
                        if path_variance > 0.5:  # High path variance indicates instability
                            consistency_score *= 0.8
                    except:
                        consistency_score *= 0.9  # Small penalty if calculation fails
                        
                return consistency_score
            else:
                return 1.0  # No data to validate, assume consistent
        except Exception as e:
            cloudlog.error(f"Error in path consistency check: {e}")
            return 0.9  # Default to slightly reduced confidence on error
    
    def _check_curvature_smoothness(self, model_output: Dict[str, Any], v_ego: float, current_curvature: float) -> float:
        """Check if curvature prediction is smooth and reasonable"""
        try:
            if 'desiredCurvature' in model_output:
                desired_curvature = model_output['desiredCurvature']
                
                # Check for excessive curvature changes
                if hasattr(desired_curvature, '__iter__') and len(desired_curvature) > 1:
                    # Check rate of change of curvature (curvature derivative)
                    curvature_changes = np.diff(desired_curvature)
                    max_change = max(np.abs(curvature_changes)) if len(curvature_changes) > 0 else 0
                    
                    # If curvature is changing too rapidly, reduce confidence
                    if max_change > 0.01:  # Threshold for excessive change
                        return 0.7
                    elif max_change > 0.005:
                        return 0.85
                else:
                    # Single curvature value, check if it's reasonable
                    if abs(desired_curvature) > 0.1 and v_ego > 15.0:  # High curvature at high speed
                        return 0.6  # Reduce confidence for potentially unsafe curvature at high speed
                    elif abs(desired_curvature) > 0.05:
                        return 0.8  # Moderate reduction for high curvature
                        
            return 1.0  # Curvature is smooth and reasonable
        except Exception as e:
            cloudlog.error(f"Error in curvature smoothness check: {e}")
            return 0.9  # Default to slightly reduced confidence on error
    
    def _check_temporal_consistency(self, model_output: Dict[str, Any]) -> float:
        """Check temporal consistency of model predictions"""
        try:
            # If we have historical data, check for consistency
            if len(self.confidence_history) > 10:  # Need at least some history
                recent_confidence = self.confidence_history[-10:]  # Last 10 readings
                avg_confidence = np.mean(recent_confidence)
                confidence_std = np.std(recent_confidence)
                
                # High standard deviation indicates inconsistent confidence
                if confidence_std > 0.3:
                    return 0.7
                elif confidence_std > 0.2:
                    return 0.85
                elif abs(avg_confidence - self.confidence_history[-1]) > 0.4:
                    # Current reading very different from recent average
                    return 0.8
            return 1.0
        except Exception as e:
            cloudlog.error(f"Error in temporal consistency check: {e}")
            return 0.9  # Default to slightly reduced confidence on error
    
    def _calculate_enhanced_confidence(self, base_confidence: float, path_consistency: float, 
                                     curvature_smoothness: float, temporal_consistency: float, 
                                     v_ego: float) -> float:
        """Calculate enhanced confidence based on all validation factors"""
        # Weight different factors based on their importance
        # At higher speeds, path consistency and curvature smoothness are more critical
        speed_factor = min(1.0, v_ego / 25.0)  # Normalize to 25 m/s (about 90 km/h)
        
        weights = {
            'base': 0.4,
            'path': 0.2 + 0.1 * speed_factor,  # Higher weight at higher speeds
            'curvature': 0.2 + 0.1 * speed_factor,  # Higher weight at higher speeds
            'temporal': 0.2
        }
        
        # Calculate weighted confidence
        enhanced_confidence = (
            weights['base'] * base_confidence +
            weights['path'] * path_consistency +
            weights['curvature'] * curvature_smoothness +
            weights['temporal'] * temporal_consistency
        )
        
        # Clamp to valid range
        enhanced_confidence = max(0.0, min(1.0, enhanced_confidence))
        
        return enhanced_confidence
    
    def get_confidence_trend(self) -> Dict[str, Any]:
        """Get trend information about confidence over time"""
        if not self.confidence_history:
            return {
                'trend': 'stable',
                'current_confidence': 1.0,
                'average_confidence': 1.0,
                'volatility': 0.0
            }
        
        recent = self.confidence_history[-10:] if len(self.confidence_history) >= 10 else self.confidence_history
        current_conf = recent[-1] if recent else 1.0
        avg_conf = np.mean(recent)
        volatility = np.std(recent)
        
        # Determine trend
        if len(recent) >= 5:
            early_avg = np.mean(recent[:len(recent)//2])
            if current_conf > early_avg + 0.1:
                trend = 'improving'
            elif current_conf < early_avg - 0.1:
                trend = 'degrading'
            else:
                trend = 'stable'
        else:
            trend = 'insufficient_data'
        
        return {
            'trend': trend,
            'current_confidence': current_conf,
            'average_confidence': avg_conf,
            'volatility': volatility
        }