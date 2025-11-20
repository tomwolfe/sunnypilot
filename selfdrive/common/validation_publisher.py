#!/usr/bin/env python3
"""
Simple Validation Publisher for Sunnypilot
Publisher for validation metrics with basic analysis and functionality
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from selfdrive.common.enhanced_validation import SimpleValidation


class SimpleValidationPublisher:
    """Simple validation publisher with basic metrics and functionality"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # Initialize validation data with basic metrics
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
        self.max_previous_states = 10  # Reduced for basic implementation
        self.publish_counter = 0
        self.publish_threshold = 5  # Publish less frequently to reduce overhead
        self.last_published_data = {}

        # Initialize simple validation for basic analysis
        self.simple_validator = SimpleValidation()

        # Track basic metrics
        self.performance_metrics = {
            'avg_processing_time': 0.0,
            'processing_time_samples': 0
        }

    def update_validation_metrics(self):
        """Update validation metrics with basic analysis"""
        start_time = time.time()

        try:
            self.sm.update(0)

            # Initialize values
            lead_confidences = []
            lane_confidences = []

            # Calculate lead confidence metrics with basic analysis
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
                    for lead in model_data.leadsV3:
                        if hasattr(lead, 'confidence'):
                            lead_confidences.append(lead.confidence)

                # Calculate lane confidence metrics with basic checks
                if hasattr(model_data, 'laneLineProbs'):
                    lane_probs = model_data.laneLineProbs
                    if lane_probs:
                        lane_confidences.extend(lane_probs)

            # Calculate averages
            self.validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences) if lead_confidences else 0.0
            self.validation_data['leadConfidenceMax'] = max(lead_confidences) if lead_confidences else 0.0
            self.validation_data['laneConfidenceAvg'] = sum(lane_confidences) / len(lane_confidences) if lane_confidences else 0.0

            # Update previous states for temporal consistency calculation
            current_state = dict(self.validation_data)
            if len(self.previous_states) >= self.max_previous_states:
                self.previous_states = self.previous_states[1:]
            self.previous_states.append(current_state)

            # Calculate simple temporal consistency
            if len(self.previous_states) > 1:
                # Calculate consistency as average difference from previous states
                consistency_sum = 0.0
                for prev_state in self.previous_states[-3:]:  # Check last 3 states
                    consistency_sum += abs(current_state.get('overallConfidence', 0) - prev_state.get('overallConfidence', 0))
                temporal_consistency = 1.0 - min(0.9, consistency_sum / len(self.previous_states[-3:]))
                self.validation_data['temporalConsistency'] = temporal_consistency
            else:
                self.validation_data['temporalConsistency'] = 1.0

            # Use simple validation from our SimpleValidation class
            lead_valid, lead_conf = self.simple_validator.validate_lead_detection(
                {'confidence': self.validation_data['leadConfidenceAvg']}
            ) if lead_confidences else (False, 0.0)

            lane_valid, lane_conf = self.simple_validator.validate_lane_detection(
                self.validation_data['laneConfidenceAvg'], self.validation_data['laneConfidenceAvg']
            )

            temporal_valid = self.simple_validator.validate_temporal_consistency(
                self.validation_data['temporalConsistency'], [self.validation_data['temporalConsistency']] * 5
            )

            # Get overall safety score
            overall_score = self.simple_validator.get_overall_safety_score(
                lead_valid, lane_valid, temporal_valid, lead_conf, lane_conf
            )

            # Update our validation data with results
            self.validation_data.update(overall_score)

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

        # Update performance metrics
        processing_time = time.time() - start_time
        self.performance_metrics['processing_time_samples'] += 1
        avg_time = self.performance_metrics['avg_processing_time']
        new_samples = self.performance_metrics['processing_time_samples']
        self.performance_metrics['avg_processing_time'] = (
            (avg_time * (new_samples - 1) + processing_time) / new_samples
        )

    def should_publish(self) -> bool:
        """Basic logic for determining if metrics should be published"""
        # Publish every N cycles for basic updates
        self.publish_counter = (self.publish_counter + 1) % self.publish_threshold
        return self.publish_counter == 0

    def publish(self):
        """Basic publishing with error handling"""
        try:
            self.update_validation_metrics()

            if not self.should_publish():
                return

            dat = messaging.new_message('validationMetrics')
            validation = dat.validationMetrics

            # Set validation metrics with basic error handling
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
            self.last_published_data = dict(self.validation_data)

        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")


def main():
    """Main validation publisher loop with basic functionality"""
    validation_publisher = SimpleValidationPublisher()
    rk = Ratekeeper(10.0)  # 10Hz for basic validation

    cloudlog.info("Simple validation publisher starting...")

    while True:
        validation_publisher.publish()
        rk.keep_time()


if __name__ == "__main__":
    main()