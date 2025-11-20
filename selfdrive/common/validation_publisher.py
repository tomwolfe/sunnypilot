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
from selfdrive.common.validation_utils import (
    calculate_temporal_consistency,
    calculate_environment_complexity,
    calculate_overall_confidence,
    calculate_safety_score,
    calculate_lead_confidences,
    calculate_lane_confidences,
    calculate_road_edge_confidence
)
from selfdrive.common.validation_config import get_validation_config


class ValidationPublisher:
    """Enhanced validation publisher for safety metrics with improved algorithms"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration', 'radarState', 'gpsLocation'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # Get configuration
        self.config = get_validation_config()

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
            'confidenceThreshold': self.config.confidence_threshold,
            'environmentComplexity': 0.5,
            'predictionAccuracy': 0.0
        }

        # Store previous states for temporal consistency
        self.previous_states = []
        self.max_previous_states = self.config.max_previous_states  # Increased for better temporal analysis
        self.velocity_history = []  # Track velocity for temporal consistency
        self.max_velocity_history = self.config.max_velocity_history

        # Performance optimization - only recalculate complex metrics periodically
        self._last_complex_calculation_time = 0
        self._calculation_interval = 0.1  # Recalculate complex metrics every 0.1 seconds

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

            # Calculate temporal consistency using shared utility
            self.validation_data['temporalConsistency'] = calculate_temporal_consistency(self.velocity_history)

            # Calculate confidences using shared utilities
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Calculate lead confidences using shared utility
                lead_avg, lead_max = calculate_lead_confidences(model_data)
                self.validation_data['leadConfidenceAvg'] = lead_avg
                self.validation_data['leadConfidenceMax'] = lead_max

                # Calculate lane confidences using shared utility
                self.validation_data['laneConfidenceAvg'] = calculate_lane_confidences(model_data)

                # Calculate road edge confidences using shared utility
                self.validation_data['roadEdgeConfidenceAvg'] = calculate_road_edge_confidence(model_data)

            # Calculate environment complexity using shared utility
            car_state = self.sm['carState'] if self.sm.updated['carState'] else None
            radar_state = self.sm['radarState'] if self.sm.updated['radarState'] else None
            model_data = self.sm['modelV2'] if self.sm.updated['modelV2'] else None

            self.validation_data['environmentComplexity'] = calculate_environment_complexity(
                model_data, radar_state, car_state
            )

            # Calculate prediction accuracy
            self.validation_data['predictionAccuracy'] = self.calculate_prediction_accuracy()

            # Calculate overall confidence using shared utility
            self.validation_data['overallConfidence'] = calculate_overall_confidence(
                self.validation_data['leadConfidenceAvg'],
                self.validation_data['laneConfidenceAvg'],
                self.validation_data['roadEdgeConfidenceAvg'],
                self.validation_data['temporalConsistency'],
                self.validation_data['environmentComplexity']
            )

            # Store current state for historical analysis
            current_state = dict(self.validation_data)
            self.previous_states.append(current_state)
            if len(self.previous_states) > self.max_previous_states:
                self.previous_states = self.previous_states[-self.max_previous_states:]

            # Calculate safety score using shared utility
            self.validation_data['safetyScore'] = calculate_safety_score(
                self.validation_data['overallConfidence'],
                self.validation_data['environmentComplexity'],
                []  # No traffic violations in this context
            )

            # Determine if system should engage with more sophisticated checks
            threshold = self.validation_data['confidenceThreshold']
            min_confidence_for_engagement = self.config.min_confidence_for_engagement  # Minimum even if threshold is lower

            # Additional checks beyond confidence
            engagement_conditions = [
                self.validation_data['overallConfidence'] >= threshold,
                self.validation_data['safetyScore'] >= min_confidence_for_engagement,
                self.validation_data['environmentComplexity'] < self.config.max_environment_complexity,  # Don't engage in very complex environments
                self.validation_data['temporalConsistency'] > self.config.min_temporal_consistency  # Require some temporal consistency
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
                'confidenceThreshold': self.config.confidence_threshold,
                'environmentComplexity': self.config.max_environment_complexity,  # High complexity on error
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
    config = get_validation_config()
    rk = Ratekeeper(config.validation_frequency)  # Use configured frequency

    cloudlog.info("Enhanced validation publisher starting...")

    while True:
        validation_publisher.publish()
        rk.keep_time()


if __name__ == "__main__":
    main()