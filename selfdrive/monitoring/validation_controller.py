#!/usr/bin/env python3
"""
Validation Controller for Sunnypilot
Coordinates validation systems with error handling
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog


class ValidationController:
    """Validation controller that coordinates validation systems"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration', 'cameraOdometry'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # State tracking
        self.velocity_history = []
        self.max_velocity_history = 10

    def update_validation_systems(self):
        """Update validation systems based on sensor data"""
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

            # Calculate temporal consistency using history
            temporal_consistency = 1.0
            if len(self.velocity_history) > 1:
                recent_velocities = self.velocity_history[-3:] if len(self.velocity_history) > 3 else self.velocity_history
                # Calculate consistency as variance of recent velocities
                avg_velocity = sum(recent_velocities) / len(recent_velocities)
                variance = sum((v - avg_velocity) ** 2 for v in recent_velocities) / len(recent_velocities)
                # Higher consistency when variance is low (velocities are stable)
                temporal_consistency = max(0.1, 1.0 - min(0.9, variance))

            # Process model data for validation
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
                'confidenceThreshold': 0.6
            }

            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Extract lead information
                lead_confidences = []
                if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
                    for lead in model_data.leadsV3:
                        if hasattr(lead, 'confidence'):
                            lead_confidences.append(lead.confidence)

                if lead_confidences:
                    validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences)
                    validation_data['leadConfidenceMax'] = max(lead_confidences)

                # Calculate lane confidence metrics
                lane_confidences = []
                if hasattr(model_data, 'laneLineProbs'):
                    lane_probs = model_data.laneLineProbs
                    if lane_probs:
                        lane_confidences.extend(lane_probs)

                if lane_confidences:
                    validation_data['laneConfidenceAvg'] = sum(lane_confidences) / len(lane_confidences)

                # Calculate road edge confidence if available
                if hasattr(model_data, 'roadEdgeStds') and model_data.roadEdgeStds:
                    road_edge_confidences = [1.0 - std for std in model_data.roadEdgeStds if std is not None]
                    if road_edge_confidences:
                        validation_data['roadEdgeConfidenceAvg'] = sum(road_edge_confidences) / len(road_edge_confidences)

                # Calculate overall confidence as weighted average
                weights = [0.3, 0.3, 0.2, 0.2]  # lead, lane, road_edge, temporal consistency
                confidences = [
                    validation_data['leadConfidenceAvg'],
                    validation_data['laneConfidenceAvg'],
                    validation_data['roadEdgeConfidenceAvg'],
                    validation_data['temporalConsistency']
                ]

                validation_data['overallConfidence'] = sum(w * c for w, c in zip(weights, confidences))

                # Determine if system should engage based on confidence threshold
                threshold = validation_data['confidenceThreshold']
                validation_data['systemShouldEngage'] = validation_data['overallConfidence'] >= threshold
                validation_data['isValid'] = (validation_data['systemShouldEngage'] and
                                           validation_data['overallConfidence'] > 0.1)

            return validation_data
        except Exception as e:
            cloudlog.error(f"Error in validation controller: {e}")
            # Return safe defaults on error
            return {
                'leadConfidenceAvg': 0.0,
                'leadConfidenceMax': 0.0,
                'laneConfidenceAvg': 0.0,
                'roadEdgeConfidenceAvg': 0.0,
                'overallConfidence': 0.0,
                'safetyScore': 0.0,
                'situationFactor': 0.5,
                'temporalConsistency': 0.0,
                'systemShouldEngage': False,
                'isValid': False,
                'confidenceThreshold': 0.6,
            }

    def publish_validation_metrics(self, validation_data: Dict[str, Any]):
        """Publish validation metrics with error handling"""
        try:
            dat = messaging.new_message('validationMetrics')
            validation = dat.validationMetrics

            # Set core validation metrics
            validation.leadConfidenceAvg = float(validation_data.get('leadConfidenceAvg', 0.0))
            validation.leadConfidenceMax = float(validation_data.get('leadConfidenceMax', 0.0))
            validation.laneConfidenceAvg = float(validation_data.get('laneConfidenceAvg', 0.0))
            validation.roadEdgeConfidenceAvg = float(validation_data.get('roadEdgeConfidenceAvg', 0.0))
            validation.overallConfidence = float(validation_data.get('overallConfidence', 0.0))
            validation.safetyScore = float(validation_data.get('safetyScore', 0.0))
            validation.situationFactor = float(validation_data.get('situationFactor', 0.5))
            validation.temporalConsistency = float(validation_data.get('temporalConsistency', 0.0))
            validation.systemShouldEngage = bool(validation_data.get('systemShouldEngage', False))
            validation.isValid = bool(validation_data.get('isValid', False))
            validation.confidenceThreshold = float(validation_data.get('confidenceThreshold', 0.6))

            self.pm.send('validationMetrics', dat)
        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")

    def run_step(self):
        """Run a single validation step with error handling"""
        try:
            validation_data = self.update_validation_systems()
            self.publish_validation_metrics(validation_data)
        except Exception as e:
            cloudlog.error(f"Error in validation controller step: {e}")


def main():
    """Main validation controller loop"""
    controller = ValidationController()
    rk = Ratekeeper(10.0)  # 10Hz update rate

    cloudlog.info("Validation controller starting...")

    while True:
        controller.run_step()
        rk.keep_time()


if __name__ == "__main__":
    main()