#!/usr/bin/env python3
"""
Validation Publisher for Sunnypilot
Publishes validation metrics for safety system
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog


class ValidationPublisher:
    """Validation publisher for safety metrics"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration'])
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
            'confidenceThreshold': 0.6
        }

        self.previous_states = []  # Store previous states for temporal consistency
        self.max_previous_states = 10

    def update_validation_metrics(self):
        """Update validation metrics from model and sensor data"""
        try:
            self.sm.update(0)

            # Initialize values
            lead_confidences = []
            lane_confidences = []

            # Calculate lead confidence metrics
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Extract lead information
                if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
                    for lead in model_data.leadsV3:
                        if hasattr(lead, 'confidence'):
                            lead_confidences.append(lead.confidence)

                # Calculate lane confidence metrics
                if hasattr(model_data, 'laneLineProbs'):
                    lane_probs = model_data.laneLineProbs
                    if lane_probs:
                        lane_confidences.extend(lane_probs)

            # Calculate averages
            self.validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences) if lead_confidences else 0.0
            self.validation_data['leadConfidenceMax'] = max(lead_confidences) if lead_confidences else 0.0
            self.validation_data['laneConfidenceAvg'] = sum(lane_confidences) / len(lane_confidences) if lane_confidences else 0.0

            # Calculate road edge confidence if available
            if hasattr(model_data, 'roadEdgeStds') and model_data.roadEdgeStds:
                road_edge_confidences = [1.0 - std for std in model_data.roadEdgeStds if std is not None]
                self.validation_data['roadEdgeConfidenceAvg'] = sum(road_edge_confidences) / len(road_edge_confidences) if road_edge_confidences else 0.0

            # Calculate overall confidence as weighted average
            weights = [0.3, 0.3, 0.2, 0.2]  # lead, lane, road_edge, temporal consistency
            confidences = [
                self.validation_data['leadConfidenceAvg'],
                self.validation_data['laneConfidenceAvg'],
                self.validation_data['roadEdgeConfidenceAvg'],
                self.validation_data['temporalConsistency']
            ]

            self.validation_data['overallConfidence'] = sum(w * c for w, c in zip(weights, confidences))

            # Calculate temporal consistency
            current_state = dict(self.validation_data)
            if len(self.previous_states) >= self.max_previous_states:
                self.previous_states = self.previous_states[1:]
            self.previous_states.append(current_state)

            if len(self.previous_states) > 1:
                # Calculate consistency as average difference from previous states
                consistency_sum = 0.0
                for prev_state in self.previous_states[-3:]:  # Check last 3 states
                    consistency_sum += abs(current_state.get('overallConfidence', 0) - prev_state.get('overallConfidence', 0))
                temporal_consistency = 1.0 - min(0.9, consistency_sum / len(self.previous_states[-3:]))
                self.validation_data['temporalConsistency'] = temporal_consistency
            else:
                self.validation_data['temporalConsistency'] = 1.0

            # Determine if system should engage based on confidence threshold
            threshold = self.validation_data['confidenceThreshold']
            self.validation_data['systemShouldEngage'] = self.validation_data['overallConfidence'] >= threshold
            self.validation_data['isValid'] = (self.validation_data['systemShouldEngage'] and
                                             self.validation_data['overallConfidence'] > 0.1)

        except Exception as e:
            cloudlog.error(f"Error updating validation metrics: {e}")
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
                'confidenceThreshold': 0.6
            })

    def publish(self):
        """Publish validation metrics"""
        try:
            self.update_validation_metrics()

            dat = messaging.new_message('validationMetrics')
            validation = dat.validationMetrics

            # Set validation metrics
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


def main():
    """Main validation publisher loop"""
    validation_publisher = ValidationPublisher()
    rk = Ratekeeper(10.0)  # 10Hz update rate

    cloudlog.info("Validation publisher starting...")

    while True:
        validation_publisher.publish()
        rk.keep_time()


if __name__ == "__main__":
    main()