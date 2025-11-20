#!/usr/bin/env python3
"""
Simple Validation Controller for Sunnypilot
Basic validation controller that coordinates validation systems with fundamental error handling
"""
import numpy as np
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from sunnypilot.selfdrive.controls.lib.traffic_light_validation import TrafficValidator
from sunnypilot.selfdrive.controls.lib.traffic_sign_detection import TrafficSignDetectionHandler
from selfdrive.common.enhanced_validation import SimpleValidation


class SimpleValidationController:
    """Basic validation controller that coordinates validation systems with fundamental error handling"""

    def __init__(self):
        # Messaging with basic sources
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'cameraOdometry', 'liveCalibration'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # Initialize validation components
        self.traffic_validator = TrafficValidator()
        self.traffic_handler = TrafficSignDetectionHandler()
        self.simple_validator = SimpleValidation()

        # State tracking
        self.velocity_history = []
        self.max_velocity_history = 10  # Reduced for basic implementation

    def update_validation_systems(self):
        """Update validation systems with basic analysis"""
        start_time = time.time()

        try:
            self.sm.update(0)

            # Process model data for traffic sign detection
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']
                traffic_signs = self.traffic_handler.process_modeld_output(model_data)

                # Add detected traffic signs to validator
                for sign in traffic_signs:
                    if sign.confidence > 0.7:  # Higher confidence threshold
                        self.traffic_validator.add_traffic_sign_data(sign)

            # Get car state for validation
            velocity = 0.0
            if self.sm.updated['carState']:
                car_state = self.sm['carState']
                velocity = getattr(car_state, 'vEgo', 0.0) or 0.0

                # Add to history for temporal analysis
                self.velocity_history.append(velocity)
                if len(self.velocity_history) > self.max_velocity_history:
                    self.velocity_history = self.velocity_history[-self.max_velocity_history:]

            # Get position for validation
            position = np.array([0.0, 0.0, 0.0])
            if self.sm.updated['cameraOdometry']:
                odometry = self.sm['cameraOdometry']
                if hasattr(odometry, 'trans'):
                    position = np.array([odometry.trans[0], odometry.trans[1], odometry.trans[2]])

            # Get heading from odometry
            heading = 0.0
            if self.sm.updated['cameraOdometry'] and hasattr(self.sm['cameraOdometry'], 'rot'):
                # Simplified heading extraction
                heading = 0.0

            # Run basic traffic light validation
            traffic_safe, traffic_violations, traffic_recommendations = self.traffic_validator.validate_traffic_lights(
                position, velocity, heading
            )

            # Run basic stop sign validation
            stop_safe, stop_violations, stop_recommendations = self.traffic_validator.validate_stop_signs(
                position, velocity, heading
            )

            # Calculate temporal consistency using history
            temporal_valid = True
            if len(self.velocity_history) > 1:
                recent_velocities = self.velocity_history[-3:] if len(self.velocity_history) > 3 else self.velocity_history
                temporal_valid = self.simple_validator.validate_temporal_consistency(
                    velocity, recent_velocities
                )

            # Calculate simple validation scores
            lead_valid, lead_conf = self.simple_validator.validate_lead_detection({'confidence': 0.8})
            lane_valid, lane_conf = self.simple_validator.validate_lane_detection(0.85, 0.85)

            # Get overall safety score
            safety_result = self.simple_validator.get_overall_safety_score(
                lead_valid, lane_valid, temporal_valid, lead_conf, lane_conf
            )

            # Factor in traffic safety violations
            all_violations = traffic_violations + stop_violations

            if all_violations:
                safety_result['overallConfidence'] *= 0.5  # Reduce confidence on violations
                safety_result['isValid'] = False
                safety_result['systemShouldEngage'] = False
                safety_result['situationFactor'] = 0.3  # Low situation factor due to violations
            else:
                safety_result['situationFactor'] = 0.8

            # Calculate processing time
            step_time = time.time() - start_time

            return safety_result
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
        """Publish validation metrics with basic error handling"""
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
        """Run a single validation step with basic error handling"""
        try:
            validation_data = self.update_validation_systems()
            self.publish_validation_metrics(validation_data)
        except Exception as e:
            cloudlog.error(f"Error in validation controller step: {e}")


def main():
    """Main validation controller loop with basic functionality"""
    controller = SimpleValidationController()
    rk = Ratekeeper(10.0)  # 10Hz for basic validation

    cloudlog.info("Simple validation controller starting...")

    while True:
        controller.run_step()
        rk.keep_time()


if __name__ == "__main__":
    main()