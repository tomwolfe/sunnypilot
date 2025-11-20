#!/usr/bin/env python3
"""
Shared utilities for validation systems in Sunnypilot
Contains common functions used across validation components
"""
from typing import List, Dict, Any
import numpy as np


def calculate_temporal_consistency(velocity_history: List[float], max_history: int = 10) -> float:
    """Calculate temporal consistency based on velocity history"""
    if len(velocity_history) <= 1:
        return 1.0

    recent_velocities = velocity_history[-10:] if len(velocity_history) > 10 else velocity_history
    if len(recent_velocities) <= 1:
        return 1.0

    velocity_changes = [abs(recent_velocities[i] - recent_velocities[i-1])
                       for i in range(1, len(recent_velocities))]
    avg_change = sum(velocity_changes) / len(velocity_changes) if velocity_changes else 0.0
    # Lower average change means higher temporal consistency
    temporal_consistency = max(0.1, 1.0 - min(0.9, avg_change * 3))  # Adjust scale factor as needed
    return temporal_consistency


def calculate_environment_complexity(model_data: Any, radar_state: Any, car_state: Any) -> float:
    """Calculate environment complexity based on sensor inputs"""
    complexity = 0.3  # Base complexity

    # Check radar state for lead vehicles
    if radar_state and hasattr(radar_state, 'leadOne') and radar_state.leadOne:
        lead = radar_state.leadOne
        if lead and hasattr(lead, 'dRel') and lead.dRel < 50:  # Within 50m
            # Closer leads increase complexity
            complexity += max(0.0, min(0.4, (50 - lead.dRel) / 50 * 0.4))

    # Check car state for current conditions
    if car_state:
        if hasattr(car_state, 'vEgo') and car_state.vEgo > 25:  # High speed increases complexity
            complexity = min(0.9, complexity + 0.25)

    # Check model data for lane conditions
    if model_data:
        if hasattr(model_data, 'laneLineProbs'):
            # Highly curved lanes increase complexity
            if hasattr(model_data, 'laneLineAngles') and len(model_data.laneLineAngles) >= 4:
                curvature = abs(model_data.laneLineAngles[3] - model_data.laneLineAngles[0])
                if curvature > 0.3:
                    complexity = min(0.9, complexity + 0.3)

    return min(1.0, complexity)


def calculate_overall_confidence(lead_confidence_avg: float, lane_confidence_avg: float,
                                road_edge_confidence_avg: float, temporal_consistency: float,
                                environment_complexity: float) -> float:
    """Calculate overall confidence with sophisticated weighting"""
    base_weights = [0.25, 0.25, 0.15, 0.1, 0.15]  # lead, lane, road_edge, temporal, situation
    complexity_factor = 1.0 - (environment_complexity * 0.3)  # Reduce weights in complex environments

    weighted_confidences = [
        lead_confidence_avg * base_weights[0] * complexity_factor,
        lane_confidence_avg * base_weights[1] * complexity_factor,
        road_edge_confidence_avg * base_weights[2] * complexity_factor,
        temporal_consistency * base_weights[3],
        1.0 * base_weights[4] * complexity_factor  # situation factor
    ]

    return sum(weighted_confidences)


def calculate_safety_score(overall_confidence: float, environment_complexity: float, 
                          traffic_violations: List[str]) -> float:
    """Calculate safety score considering multiple factors"""
    safety_score = overall_confidence
    env_factor = 1.0 - (environment_complexity * 0.2)  # Lower safety in complex environments
    violation_penalty = len(traffic_violations) * 0.1  # Small penalty for violations
    safety_score = max(0.0, min(1.0, safety_score * env_factor - violation_penalty))
    return safety_score


def calculate_lead_confidences(model_data) -> tuple:
    """Calculate lead confidence metrics from model data"""
    if not (model_data and hasattr(model_data, 'leadsV3') and model_data.leadsV3):
        return 0.0, 0.0  # avg, max

    lead_confidences = []
    for lead in model_data.leadsV3:
        if hasattr(lead, 'confidence') and hasattr(lead, 'dRel'):
            # Apply distance-based confidence weighting
            distance_factor = 1.0
            if lead.dRel < 10:  # Very close
                distance_factor = 0.8
            elif lead.dRel > 100:  # Very far
                distance_factor = 0.6
            lead_confidences.append(lead.confidence * distance_factor)

    if not lead_confidences:
        return 0.0, 0.0

    avg_confidence = sum(lead_confidences) / len(lead_confidences)
    max_confidence = max(lead_confidences)
    return avg_confidence, max_confidence


def calculate_lane_confidences(model_data) -> float:
    """Calculate lane confidence metrics from model data"""
    if not (model_data and hasattr(model_data, 'laneLineProbs') and len(model_data.laneLineProbs) >= 4):
        return 0.0

    # Consider only the most reliable lane lines
    reliable_probs = [p for p in model_data.laneLineProbs[:4] if p is not None and p >= 0.1]
    if not reliable_probs:
        return 0.0

    # Apply distance weighting to lane confidence
    distance_weights = [1.0, 0.9, 0.8, 0.7]  # More weight to closer lane lines
    weighted_sum = sum(p * w for p, w in zip(reliable_probs, distance_weights[:len(reliable_probs)]))
    weight_sum = sum(distance_weights[:len(reliable_probs)])
    avg_confidence = weighted_sum / weight_sum if weight_sum > 0 else 0.0
    return avg_confidence


def calculate_road_edge_confidence(model_data) -> float:
    """Calculate road edge confidence from model data"""
    if not (model_data and hasattr(model_data, 'roadEdgeStds') and model_data.roadEdgeStds):
        return 0.0

    # Calculate confidence based on standard deviation (inverted)
    road_edge_confidences = []
    for std in model_data.roadEdgeStds:
        if std is not None and std >= 0:
            # Lower std = higher confidence
            conf = max(0.0, min(1.0, 1.0 - std))
            road_edge_confidences.append(conf)

    if not road_edge_confidences:
        return 0.0

    return sum(road_edge_confidences) / len(road_edge_confidences)