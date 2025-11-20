#!/usr/bin/env python3
"""
Enhanced Validation Controller for Sunnypilot
Coordinates validation systems with improved algorithms and comprehensive error handling
"""
import time
from typing import Dict, Any
import numpy as np

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from selfdrive.common.validation_utils import calculate_temporal_consistency, calculate_safety_score


class ValidationController:
    """Enhanced validation controller that coordinates validation systems"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration', 'cameraOdometry',
                                      'radarState', 'gpsLocation', 'liveParameters'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # State tracking
        self.velocity_history = []
        self.max_velocity_history = 50  # Increased for better temporal analysis
        self.validation_history = []    # Track validation metrics over time
        self.max_validation_history = 20

    def update_validation_systems(self):
        """Update validation systems based on sensor data with enhanced algorithms"""
        try:
            self.sm.update(0)

            # Get car state for validation
            velocity = 0.0
            if self.sm.updated['carState']:
                car_state = self.sm['carState']
                velocity = getattr(car_state, 'vEgo', 0.0) or 0.0

                # Add to history for temporal analysis
                self.velocity_history.append(velocity)
                if len(self.velocity_history) > self.max_velocity_history:
                    self.velocity_history = self.velocity_history[-self.max_velocity_history:]

            # Calculate environment complexity
            environment_complexity = self._calculate_environment_complexity()

            # Calculate temporal consistency using velocity history
            temporal_consistency = 1.0
            if len(self.velocity_history) > 1:
                recent_velocities = self.velocity_history[-10:] if len(self.velocity_history) > 10 else self.velocity_history
                if len(recent_velocities) > 1:
                    velocity_changes = [abs(recent_velocities[i] - recent_velocities[i-1])
                                       for i in range(1, len(recent_velocities))]
                    avg_change = sum(velocity_changes) / len(velocity_changes) if velocity_changes else 0.0
                    # Lower average change means higher temporal consistency
                    temporal_consistency = max(0.1, 1.0 - min(0.9, avg_change * 3))  # Adjust scale factor as needed

            # Process model data for validation with enhanced algorithms
            validation_data = {
                'leadConfidenceAvg': 0.0,
                'leadConfidenceMax': 0.0,
                'laneConfidenceAvg': 0.0,
                'roadEdgeConfidenceAvg': 0.0,
                'overallConfidence': 0.0,
                'safetyScore': 0.0,
                'situationFactor': 1.0,
                'temporalConsistency': temporal_consistency,
                'systemShouldEngage': False,
                'isValid': False,
                'confidenceThreshold': 0.7,  # Increased for safety
                'environmentComplexity': environment_complexity,
                'predictionAccuracy': 0.85  # Default high accuracy
            }

            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Extract lead information with more sophisticated validation
                lead_confidences = []
                if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
                    for lead in model_data.leadsV3:
                        if hasattr(lead, 'confidence') and hasattr(lead, 'dRel'):
                            # Apply distance-based confidence weighting
                            distance_factor = 1.0
                            if lead.dRel < 10:  # Very close
                                distance_factor = 0.8
                            elif lead.dRel > 100:  # Very far
                                distance_factor = 0.6
                            lead_confidences.append(lead.confidence * distance_factor)

                if lead_confidences:
                    validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences)
                    validation_data['leadConfidenceMax'] = max(lead_confidences)

                # Calculate lane confidence metrics with better validation
                lane_confidences = []
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

                if lane_confidences:
                    validation_data['laneConfidenceAvg'] = sum(lane_confidences) / len(lane_confidences)

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
                        validation_data['roadEdgeConfidenceAvg'] = sum(road_edge_confidences) / len(road_edge_confidences)

                # Calculate overall confidence with more sophisticated weighting
                base_weights = [0.25, 0.25, 0.15, 0.1, 0.15]  # lead, lane, road_edge, temporal, situation
                complexity_factor = 1.0 - (environment_complexity * 0.3)  # Reduce weights in complex environments

                weighted_confidences = [
                    validation_data['leadConfidenceAvg'] * base_weights[0] * complexity_factor,
                    validation_data['laneConfidenceAvg'] * base_weights[1] * complexity_factor,
                    validation_data['roadEdgeConfidenceAvg'] * base_weights[2] * complexity_factor,
                    validation_data['temporalConsistency'] * base_weights[3],
                    validation_data['situationFactor'] * base_weights[4] * complexity_factor
                ]

                validation_data['overallConfidence'] = sum(weighted_confidences)

            # Calculate safety score considering environment complexity
            validation_data['safetyScore'] = calculate_safety_score(
                validation_data['overallConfidence'],
                validation_data['environmentComplexity'],
                []  # No traffic violations in this context
            )

            # Determine if system should engage with more sophisticated checks
            threshold = validation_data['confidenceThreshold']
            min_confidence_for_engagement = 0.6  # Minimum even if threshold is lower

            # Additional checks beyond confidence
            engagement_conditions = [
                validation_data['overallConfidence'] >= threshold,
                validation_data['safetyScore'] >= min_confidence_for_engagement,
                validation_data['environmentComplexity'] < 0.8,  # Don't engage in very complex environments
                validation_data['temporalConsistency'] > 0.3  # Require some temporal consistency
            ]

            validation_data['systemShouldEngage'] = all(engagement_conditions)
            validation_data['isValid'] = (
                validation_data['systemShouldEngage'] and
                validation_data['overallConfidence'] > min_confidence_for_engagement
            )

            # Store validation data for temporal consistency analysis
            self.validation_history.append(dict(validation_data))
            if len(self.validation_history) > self.max_validation_history:
                self.validation_history = self.validation_history[-self.max_validation_history:]

            return validation_data
        except Exception as e:
            cloudlog.error(f"Error in validation controller: {e}")
            cloudlog.exception(e)  # Log the full exception trace
            # Return safe defaults on error
            return {
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
                'environmentComplexity': 0.9,  # High complexity on error
                'predictionAccuracy': 0.0
            }

    def _calculate_environment_complexity(self) -> float:
        """Calculate environment complexity based on various sensor inputs"""
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
            if hasattr(car_state, 'vEgo') and car_state.vEgo > 25:  # High speed increases complexity
                complexity = min(0.9, complexity + 0.25)

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

    def publish_validation_metrics(self, validation_data: Dict[str, Any]):
        """Publish validation metrics with enhanced error handling"""
        try:
            dat = messaging.new_message('validationMetrics')
            validation = dat.validationMetrics

            # Set validation metrics with enhanced safety
            validation.leadConfidenceAvg = float(max(0.0, min(1.0, validation_data.get('leadConfidenceAvg', 0.0))))
            validation.leadConfidenceMax = float(max(0.0, min(1.0, validation_data.get('leadConfidenceMax', 0.0))))
            validation.laneConfidenceAvg = float(max(0.0, min(1.0, validation_data.get('laneConfidenceAvg', 0.0))))
            validation.roadEdgeConfidenceAvg = float(max(0.0, min(1.0, validation_data.get('roadEdgeConfidenceAvg', 0.0))))
            validation.overallConfidence = float(max(0.0, min(1.0, validation_data.get('overallConfidence', 0.0))))
            validation.safetyScore = float(max(0.0, min(1.0, validation_data.get('safetyScore', 0.0))))
            validation.situationFactor = float(max(0.0, min(1.0, validation_data.get('situationFactor', 1.0))))
            validation.temporalConsistency = float(max(0.0, min(1.0, validation_data.get('temporalConsistency', 0.0))))
            validation.systemShouldEngage = bool(validation_data.get('systemShouldEngage', False))
            validation.isValid = bool(validation_data.get('isValid', False))
            validation.confidenceThreshold = float(max(0.0, min(1.0, validation_data.get('confidenceThreshold', 0.7))))

            # Add new fields that weren't in the original capnp but are now calculated
            # (These would need to be added to the capnp schema to actually be sent,
            # but we'll include them in the data structure for consistency)

            self.pm.send('validationMetrics', dat)
        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")
            cloudlog.exception(e)  # Log the full exception trace

    def run_step(self):
        """Run a single validation step with enhanced error handling"""
        try:
            validation_data = self.update_validation_systems()
            self.publish_validation_metrics(validation_data)
        except Exception as e:
            cloudlog.error(f"Error in validation controller step: {e}")
            cloudlog.exception(e)  # Log the full exception trace


def main():
    """Main validation controller loop"""
    controller = ValidationController()
    rk = Ratekeeper(20.0)  # Increased to 20Hz for more responsive validation

    cloudlog.info("Enhanced validation controller starting...")

    while True:
        controller.run_step()
        rk.keep_time()


if __name__ == "__main__":
    main()