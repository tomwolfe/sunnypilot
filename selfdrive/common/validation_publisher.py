#!/usr/bin/env python3
"""
Enhanced Validation Publisher for Sunnypilot
Publishes comprehensive validation metrics for safety system with improved algorithms
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
import numpy as np


class ValidationPublisher:
    """Enhanced validation publisher for safety metrics with improved algorithms"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration', 'radarState', 'gpsLocation'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # Initialize validation data
        self.validation_data = {
            'leadConfidenceAvg': 0.0,
            'leadConfidenceMax': 0.0,
            'laneConfidenceAvg': 0.0,
            'roadEdgeConfidenceAvg': 0.0,
            'overallConfidence': 0.0,
            'safetyScore': 0.0,
            'situationFactor': 1.0,
            'temporalConsistency': 0.0,
            'systemShouldEngage': False,
            'isValid': False,
            'confidenceThreshold': 0.7,  # Increased for safety
            'environmentComplexity': 0.5,
            'predictionAccuracy': 0.0
        }

        # Store previous states for temporal consistency
        self.previous_states = []
        self.max_previous_states = 20  # Increased for better temporal analysis
        self.velocity_history = []  # Track velocity for temporal consistency
        self.max_velocity_history = 50

    def calculate_environment_complexity(self):
        """Calculate environment complexity based on sensor inputs"""
        complexity = 0.3  # Base complexity

        # Check radar state for lead vehicles
        if self.sm.updated['radarState']:
            radar_state = self.sm['radarState']
            if hasattr(radar_state, 'leadOne') and radar_state.leadOne:
                lead = radar_state.leadOne
                if lead and hasattr(lead, 'dRel') and lead.dRel < 50:  # Within 50m
                    # Closer leads increase complexity
                    complexity += max(0.0, min(0.4, (50 - lead.dRel) / 50 * 0.4))

        # Check car state for current conditions
        if self.sm.updated['carState']:
            car_state = self.sm['carState']
            if hasattr(car_state, 'vEgo') and car_state.vEgo > 20:  # High speed increases complexity
                complexity = min(0.9, complexity + 0.2)

        # Check model data for lane conditions
        if self.sm.updated['modelV2']:
            model_data = self.sm['modelV2']
            if hasattr(model_data, 'laneLineProbs'):
                # Highly curved lanes increase complexity
                if hasattr(model_data, 'laneLineAngles') and len(model_data.laneLineAngles) >= 4:
                    curvature = abs(model_data.laneLineAngles[3] - model_data.laneLineAngles[0])
                    if curvature > 0.3:
                        complexity = min(0.9, complexity + 0.3)

        return min(1.0, complexity)

    def calculate_prediction_accuracy(self):
        """Calculate prediction accuracy based on historical performance"""
        # For now, return a placeholder - a real implementation would compare predictions with actual outcomes
        return 0.85  # High accuracy as default, would be calculated in real system

    def update_validation_metrics(self):
        """Update validation metrics from model and sensor data with enhanced algorithms"""
        try:
            self.sm.update(0)

            # Update velocity history for temporal consistency
            if self.sm.updated['carState'] and hasattr(self.sm['carState'], 'vEgo'):
                velocity = self.sm['carState'].vEgo
                self.velocity_history.append(velocity)
                if len(self.velocity_history) > self.max_velocity_history:
                    self.velocity_history = self.velocity_history[-self.max_velocity_history:]

            # Initialize values
            lead_confidences = []
            lane_confidences = []

            # Calculate lead confidence metrics
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Extract lead information with more sophisticated validation
                if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
                    for lead in model_data.leadsV3:
                        if hasattr(lead, 'confidence') and hasattr(lead, 'dRel'):
                            # Discount confidence for very close or very far leads
                            distance_factor = 1.0
                            if lead.dRel < 10:  # Very close
                                distance_factor = 0.8
                            elif lead.dRel > 100:  # Very far
                                distance_factor = 0.7
                            lead_confidences.append(lead.confidence * distance_factor)

                # Calculate lane confidence metrics with better validation
                if hasattr(model_data, 'laneLineProbs') and len(model_data.laneLineProbs) >= 4:
                    # Consider only the most reliable lane lines
                    reliable_probs = [p for p in model_data.laneLineProbs[:4] if p is not None and p >= 0.1]
                    if reliable_probs:
                        # Apply distance weighting to lane confidence
                        distance_weights = [1.0, 0.9, 0.8, 0.7]  # More weight to closer lane lines
                        weighted_sum = sum(p * w for p, w in zip(reliable_probs, distance_weights[:len(reliable_probs)]))
                        weight_sum = sum(distance_weights[:len(reliable_probs)])
                        avg_confidence = weighted_sum / weight_sum if weight_sum > 0 else 0.0
                        lane_confidences.extend([avg_confidence] * len(reliable_probs))

            # Calculate averages with fallbacks
            self.validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences) if lead_confidences else 0.0
            self.validation_data['leadConfidenceMax'] = max(lead_confidences) if lead_confidences else 0.0
            self.validation_data['laneConfidenceAvg'] = sum(lane_confidences) / len(lane_confidences) if lane_confidences else 0.0

            # Calculate road edge confidence if available with better validation
            if hasattr(model_data, 'roadEdgeStds') and model_data.roadEdgeStds:
                # Calculate confidence based on standard deviation (inverted)
                road_edge_confidences = []
                for std in model_data.roadEdgeStds:
                    if std is not None and std >= 0:
                        # Lower std = higher confidence
                        conf = max(0.0, min(1.0, 1.0 - std))
                        road_edge_confidences.append(conf)

                if road_edge_confidences:
                    self.validation_data['roadEdgeConfidenceAvg'] = sum(road_edge_confidences) / len(road_edge_confidences)

            # Calculate environment complexity
            self.validation_data['environmentComplexity'] = self.calculate_environment_complexity()

            # Calculate prediction accuracy
            self.validation_data['predictionAccuracy'] = self.calculate_prediction_accuracy()

            # Calculate overall confidence with more sophisticated weighting
            # Weight based on environment complexity and current conditions
            base_weights = [0.25, 0.25, 0.15, 0.1, 0.15]  # lead, lane, road_edge, temporal, situation
            complexity_factor = 1.0 - (self.validation_data['environmentComplexity'] * 0.3)  # Reduce weights in complex environments

            weighted_confidences = [
                self.validation_data['leadConfidenceAvg'] * base_weights[0] * complexity_factor,
                self.validation_data['laneConfidenceAvg'] * base_weights[1] * complexity_factor,
                self.validation_data['roadEdgeConfidenceAvg'] * base_weights[2] * complexity_factor,
                self.validation_data['temporalConsistency'] * base_weights[3],
                self.validation_data['situationFactor'] * base_weights[4] * complexity_factor
            ]

            self.validation_data['overallConfidence'] = sum(weighted_confidences)

            # Calculate temporal consistency using velocity history
            if len(self.velocity_history) > 1:
                # Calculate consistency based on velocity changes
                recent_velocities = self.velocity_history[-10:] if len(self.velocity_history) > 10 else self.velocity_history
                if len(recent_velocities) > 1:
                    velocity_changes = [abs(recent_velocities[i] - recent_velocities[i-1])
                                       for i in range(1, len(recent_velocities))]
                    avg_change = sum(velocity_changes) / len(velocity_changes) if velocity_changes else 0.0
                    # Lower average change means higher temporal consistency
                    temporal_consistency = max(0.1, 1.0 - min(0.9, avg_change * 2))  # Adjust scale factor as needed
                    self.validation_data['temporalConsistency'] = temporal_consistency
            else:
                self.validation_data['temporalConsistency'] = 1.0

            # Store current state for historical analysis
            current_state = dict(self.validation_data)
            self.previous_states.append(current_state)
            if len(self.previous_states) > self.max_previous_states:
                self.previous_states = self.previous_states[-self.max_previous_states:]

            # Calculate safety score considering multiple factors
            safety_score = self.validation_data['overallConfidence']
            env_factor = 1.0 - (self.validation_data['environmentComplexity'] * 0.2)  # Lower safety in complex environments
            pred_factor = self.validation_data['predictionAccuracy'] * 0.1  # Small boost for good predictions
            safety_score = max(0.0, min(1.0, safety_score * env_factor + pred_factor))
            self.validation_data['safetyScore'] = safety_score

            # Determine if system should engage with more sophisticated checks
            threshold = self.validation_data['confidenceThreshold']
            min_confidence_for_engagement = 0.6  # Minimum even if threshold is lower

            # Additional checks beyond confidence
            engagement_conditions = [
                self.validation_data['overallConfidence'] >= threshold,
                self.validation_data['safetyScore'] >= min_confidence_for_engagement,
                self.validation_data['environmentComplexity'] < 0.8,  # Don't engage in very complex environments
                self.validation_data['temporalConsistency'] > 0.3  # Require some temporal consistency
            ]

            self.validation_data['systemShouldEngage'] = all(engagement_conditions)
            self.validation_data['isValid'] = (
                self.validation_data['systemShouldEngage'] and
                self.validation_data['overallConfidence'] > min_confidence_for_engagement
            )

        except Exception as e:
            cloudlog.error(f"Error updating validation metrics: {e}")
            cloudlog.exception(e)  # Log the full exception trace
            # Set safe defaults on error
            self.validation_data.update({
                'leadConfidenceAvg': 0.0,
                'leadConfidenceMax': 0.0,
                'laneConfidenceAvg': 0.0,
                'roadEdgeConfidenceAvg': 0.0,
                'overallConfidence': 0.0,
                'safetyScore': 0.0,
                'situationFactor': 1.0,
                'temporalConsistency': 0.0,
                'systemShouldEngage': False,
                'isValid': False,
                'confidenceThreshold': 0.7,
                'environmentComplexity': 0.8,  # High complexity on error
                'predictionAccuracy': 0.0
            })

    def publish(self):
        """Publish validation metrics with enhanced error handling"""
        try:
            self.update_validation_metrics()

            dat = messaging.new_message('validationMetrics')
            validation = dat.validationMetrics

            # Set validation metrics with type safety
            validation.leadConfidenceAvg = float(self.validation_data['leadConfidenceAvg'])
            validation.leadConfidenceMax = float(self.validation_data['leadConfidenceMax'])
            validation.laneConfidenceAvg = float(self.validation_data['laneConfidenceAvg'])
            validation.roadEdgeConfidenceAvg = float(self.validation_data['roadEdgeConfidenceAvg'])
            validation.overallConfidence = float(self.validation_data['overallConfidence'])
            validation.safetyScore = float(self.validation_data['safetyScore'])
            validation.situationFactor = float(self.validation_data['situationFactor'])
            validation.temporalConsistency = float(self.validation_data['temporalConsistency'])
            validation.systemShouldEngage = bool(self.validation_data['systemShouldEngage'])
            validation.isValid = bool(self.validation_data['isValid'])
            validation.confidenceThreshold = float(self.validation_data['confidenceThreshold'])

            self.pm.send('validationMetrics', dat)

        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")
            cloudlog.exception(e)  # Log the full exception trace


def main():
    """Main validation publisher loop"""
    validation_publisher = ValidationPublisher()
    rk = Ratekeeper(20.0)  # Increased to 20Hz for more responsive validation

    cloudlog.info("Enhanced validation publisher starting...")

    while True:
        validation_publisher.publish()
        rk.keep_time()


if __name__ == "__main__":
    main()