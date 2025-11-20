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


class ValidationController:
    """Enhanced validation controller that coordinates validation systems"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration', 'cameraOdometry',
                                      'radarState', 'gpsLocation', 'liveParameters'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # Get configuration
        self.config = get_validation_config()

        # State tracking
        self.velocity_history = []
        self.max_velocity_history = self.config.max_velocity_history  # Increased for better temporal analysis
        self.validation_history = []    # Track validation metrics over time
        self.max_validation_history = self.config.max_previous_states

        # Performance optimization - only recalculate complex metrics periodically
        self._last_complex_calculation_time = 0
        self._calculation_interval = 0.1  # Recalculate complex metrics every 0.1 seconds

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

            # Calculate temporal consistency using shared utility
            temporal_consistency = calculate_temporal_consistency(self.velocity_history)

            # Calculate environment complexity using shared utility
            model_data = self.sm['modelV2'] if self.sm.updated['modelV2'] else None
            radar_state = self.sm['radarState'] if self.sm.updated['radarState'] else None
            car_state_for_complexity = self.sm['carState'] if self.sm.updated['carState'] else None
            environment_complexity = calculate_environment_complexity(
                model_data, radar_state, car_state_for_complexity
            )

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
                'confidenceThreshold': self.config.confidence_threshold,  # Increased for safety
                'environmentComplexity': environment_complexity,
                'predictionAccuracy': 0.85  # Default high accuracy
            }

            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Calculate lead confidences using shared utility
                lead_avg, lead_max = calculate_lead_confidences(model_data)
                validation_data['leadConfidenceAvg'] = lead_avg
                validation_data['leadConfidenceMax'] = lead_max

                # Calculate lane confidences using shared utility
                validation_data['laneConfidenceAvg'] = calculate_lane_confidences(model_data)

                # Calculate road edge confidences using shared utility
                validation_data['roadEdgeConfidenceAvg'] = calculate_road_edge_confidence(model_data)

                # Calculate overall confidence using shared utility
                validation_data['overallConfidence'] = calculate_overall_confidence(
                    validation_data['leadConfidenceAvg'],
                    validation_data['laneConfidenceAvg'],
                    validation_data['roadEdgeConfidenceAvg'],
                    validation_data['temporalConsistency'],
                    validation_data['environmentComplexity']
                )

            # Calculate safety score considering environment complexity
            validation_data['safetyScore'] = calculate_safety_score(
                validation_data['overallConfidence'],
                validation_data['environmentComplexity'],
                []  # No traffic violations in this context
            )

            # Determine if system should engage with more sophisticated checks
            threshold = validation_data['confidenceThreshold']
            min_confidence_for_engagement = self.config.min_confidence_for_engagement  # Minimum even if threshold is lower

            # Additional checks beyond confidence
            engagement_conditions = [
                validation_data['overallConfidence'] >= threshold,
                validation_data['safetyScore'] >= min_confidence_for_engagement,
                validation_data['environmentComplexity'] < self.config.max_environment_complexity,  # Don't engage in very complex environments
                validation_data['temporalConsistency'] > self.config.min_temporal_consistency  # Require some temporal consistency
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
                'confidenceThreshold': self.config.confidence_threshold,
                'environmentComplexity': self.config.max_environment_complexity,  # High complexity on error
                'predictionAccuracy': 0.0
            }

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
    config = get_validation_config()
    rk = Ratekeeper(config.validation_frequency)  # Use configured frequency

    cloudlog.info("Enhanced validation controller starting...")

    while True:
        controller.run_step()
        rk.keep_time()


if __name__ == "__main__":
    main()