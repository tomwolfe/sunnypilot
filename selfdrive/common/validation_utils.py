"""
Validation Utilities for Sunnypilot
Utility functions for validation calculations and metrics
"""
from typing import Dict, List, Any
import numpy as np


def calculate_temporal_consistency(current_state: Dict, previous_states: List[Dict]) -> float:
    """
    Calculate temporal consistency of validation metrics over time
    """
    if len(previous_states) < 2:
        return 1.0 if previous_states else 0.0
    
    # Calculate consistency for key metrics
    consistency_scores = []
    metrics_to_check = ['leadConfidenceAvg', 'laneConfidenceAvg', 'overallConfidence']
    
    for metric in metrics_to_check:
        if metric in current_state:
            current_value = current_state[metric]
            # Calculate variance from recent values
            recent_values = [state.get(metric, current_value) for state in previous_states[-5:]]
            if len(recent_values) > 1:
                variance = np.var(recent_values)
                # Convert variance to consistency score (higher for lower variance)
                consistency = 1.0 / (1.0 + variance * 5)  # Scale factor for appropriate range
                consistency_scores.append(consistency)
            else:
                consistency_scores.append(1.0)
    
    # Return average consistency
    return sum(consistency_scores) / len(consistency_scores) if consistency_scores else 1.0


def calculate_prediction_accuracy(predicted_values: List[float], actual_values: List[float]) -> float:
    """
    Calculate prediction accuracy based on predicted vs actual values
    """
    if not predicted_values or not actual_values or len(predicted_values) != len(actual_values):
        return 0.0
    
    # Calculate mean absolute error
    mae = sum(abs(p - a) for p, a in zip(predicted_values, actual_values)) / len(predicted_values)
    
    # Convert to accuracy score (1.0 for perfect prediction, 0.0 for very poor)
    # Assuming values are typically in 0-1 range, we use 1.0 as max error for normalization
    accuracy = max(0.0, min(1.0, 1.0 - mae))
    return accuracy


def calculate_environment_complexity(traffic_objects: List[Dict]) -> float:
    """
    Calculate environment complexity based on traffic objects
    """
    if not traffic_objects:
        return 0.3  # Low complexity by default
    
    # Weight different types of objects differently
    complexity = 0.0
    for obj in traffic_objects:
        obj_type = obj.get('type', 'unknown')
        distance = obj.get('distance', 100.0)
        
        # Weight closer objects more heavily
        weight = max(0.0, min(1.0, 30.0 / max(distance, 1.0)))
        
        if obj_type in ['traffic_light_red', 'traffic_light_yellow']:
            complexity += weight * 0.8
        elif obj_type in ['stop_sign', 'yield_sign']:
            complexity += weight * 0.7
        elif obj_type == 'pedestrian_crossing':
            complexity += weight * 0.9
        else:
            complexity += weight * 0.3
    
    return min(1.0, complexity / len(traffic_objects) * 2)  # Normalize


def calculate_safety_score(overall_confidence: float, environment_complexity: float, 
                          traffic_violations: List[str], safety_factor: float = 1.0) -> float:
    """
    Calculate overall safety score based on multiple factors
    """
    # Base safety score from overall confidence
    base_score = overall_confidence
    
    # Reduce score based on environment complexity
    complexity_factor = 1.0 - (environment_complexity * 0.2)  # Up to 20% reduction
    
    # Reduce score based on traffic violations
    violation_factor = 1.0
    if traffic_violations:
        # More violations = lower score (linear scale up to 50% reduction)
        violation_penalty = min(0.5, len(traffic_violations) * 0.1)
        violation_factor = max(0.5, 1.0 - violation_penalty)
    
    safety_score = base_score * complexity_factor * violation_factor * safety_factor
    
    return max(0.0, min(1.0, safety_score))