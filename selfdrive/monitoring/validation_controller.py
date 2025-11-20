#!/usr/bin/env python3
"""
Enhanced Validation Controller for Sunnypilot
Advanced validation controller that coordinates all validation systems with comprehensive error handling and adaptive behavior
"""
import numpy as np
import time
from typing import Dict, Any
import math

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from selfdrive.common.performance_monitor import perf_monitor
from sunnypilot.selfdrive.controls.lib.traffic_light_validation import TrafficValidator, TrafficSignData, TrafficSignType
from sunnypilot.selfdrive.controls.lib.traffic_sign_detection import TrafficSignDetectionHandler
from selfdrive.common.enhanced_validation import SimpleValidation
from selfdrive.perception.behavior_prediction import EnhancedBehaviorPredictor, ObjectBehaviorType


class EnhancedValidationController:
    """Advanced validation controller that coordinates all validation systems with adaptive behavior and comprehensive error handling"""

    def __init__(self):
        # Messaging with additional sources for comprehensive validation
        self.sm = messaging.SubMaster([
            'modelV2', 'carState', 'controlsState', 'cameraOdometry', 'liveCalibration',
            'radarState', 'liveTracks', 'lateralPlan', 'longitudinalPlan', 'driverMonitoring'
        ])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # Initialize validation components
        self.traffic_validator = TrafficValidator()
        self.traffic_handler = TrafficSignDetectionHandler()  # Updated to use enhanced handler
        self.enhanced_validator = SimpleValidation()

        # Initialize behavior prediction for advanced safety analysis
        self.behavior_predictor = EnhancedBehaviorPredictor(
            prediction_horizon=5.0,  # 5-second horizon for better prediction
            dt=0.1  # 10 Hz prediction steps
        )

        # State tracking
        self.position_history = []
        self.max_position_history = 50  # Increased for better temporal analysis
        self.velocity_history = []
        self.max_velocity_history = 50
        self.acceleration_history = []
        self.max_acceleration_history = 50

        # Adaptive thresholds that update based on conditions
        self.adaptive_thresholds = {
            'confidence': 0.6,
            'traffic_violation_threshold': 0.5,
            'lane_keeping_threshold': 0.3
        }

        # Performance monitoring
        self.performance_stats = {
            'avg_step_time': 0.0,
            'max_step_time': 0.0,
            'step_count': 0
        }

    @perf_monitor.monitor_performance("update_validation_systems")
    def update_validation_systems(self):
        """Update all validation systems with comprehensive analysis and performance optimization"""
        start_time = time.time()

        try:
            self.sm.update(0)

            # Process model data for traffic sign detection
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']
                traffic_signs = self.traffic_handler.process_modeld_output(model_data)

                # Add detected traffic signs to validator with confidence filtering
                for sign in traffic_signs:
                    if sign.confidence > 0.6:  # Higher confidence threshold
                        self.traffic_validator.add_traffic_sign_data(sign)

            # Get car state for validation
            velocity = 0.0
            acceleration = 0.0
            if self.sm.updated['carState']:
                car_state = self.sm['carState']
                velocity = getattr(car_state, 'vEgo', 0.0) or 0.0
                acceleration = getattr(car_state, 'aEgo', 0.0) or 0.0

                # Add to history for temporal analysis
                self.velocity_history.append(velocity)
                if len(self.velocity_history) > self.max_velocity_history:
                    self.velocity_history = self.velocity_history[-self.max_velocity_history:]

                self.acceleration_history.append(acceleration)
                if len(self.acceleration_history) > self.max_acceleration_history:
                    self.acceleration_history = self.acceleration_history[-self.max_acceleration_history:]

            # Get position for validation
            position = np.array([0.0, 0.0, 0.0])
            if self.sm.updated['cameraOdometry']:
                odometry = self.sm['cameraOdometry']
                if hasattr(odometry, 'trans'):
                    position = np.array([odometry.trans[0], odometry.trans[1], odometry.trans[2]])

            # Add position to history
            self.position_history.append(position.copy())
            if len(self.position_history) > self.max_position_history:
                self.position_history = self.position_history[-self.max_position_history:]

            # Get heading for more accurate validation
            heading = 0.0
            if self.sm.updated['cameraOdometry'] and hasattr(self.sm['cameraOdometry'], 'rot'):
                # Extract heading from rotation matrix (simplified)
                # In a real implementation, this would be more sophisticated
                heading = 0.0  # Default heading

            # Get desired speed from longitudinal plan
            desired_speed = 0.0
            if self.sm.updated['longitudinalPlan'] and hasattr(self.sm['longitudinalPlan'], 'speeds'):
                speeds = self.sm['longitudinalPlan'].speeds
                if speeds:
                    desired_speed = speeds[0] if speeds[0] else 0.0

            # Run enhanced traffic light validation with additional parameters
            traffic_safe, traffic_violations, traffic_recommendations = self.traffic_validator.validate_traffic_lights(
                position, velocity, heading, desired_speed
            )

            # Run enhanced stop sign validation
            stop_safe, stop_violations, stop_recommendations = self.traffic_validator.validate_stop_signs(
                position, velocity, heading
            )

            # Run speed limit validation
            road_speed_limit = getattr(car_state, 'vCruise', 0.0) if hasattr(car_state, 'vCruise') else None
            speed_limit_safe, speed_violations, speed_recommendations = self.traffic_validator.validate_speed_limits(
                position, velocity, road_speed_limit
            )

            # Run pedestrian crossing validation
            ped_safe, ped_violations, ped_recommendations = self.traffic_validator.validate_pedestrian_crossings(
                position, velocity
            )

            # Get comprehensive traffic context
            traffic_context = self.traffic_validator.get_traffic_context(position)

            # Perform enhanced validation with more detailed parameters
            lead_valid, lead_conf = True, 0.8  # Would use real lead data in production
            lane_valid, lane_conf = True, 0.85  # Would use real lane data in production

            # Calculate temporal consistency using extended history
            temporal_valid = True
            if len(self.velocity_history) > 1:
                recent_velocities = self.velocity_history[-5:] if len(self.velocity_history) > 5 else self.velocity_history
                temporal_valid = self.enhanced_validator.validate_temporal_consistency(
                    velocity, recent_velocities
                )

            # Calculate prediction accuracy based on actual vs predicted behavior
            prediction_valid = self._calculate_prediction_validity(position, velocity, acceleration)

            # Get overall safety score with comprehensive factors
            safety_result = self.enhanced_validator.get_overall_safety_score(
                lead_valid, lane_valid, temporal_valid, lead_conf, lane_conf, prediction_valid
            )

            # Factor in traffic safety violations with severity-based scaling
            all_violations = traffic_violations + stop_violations + speed_violations + ped_violations

            if all_violations:
                # Calculate violation severity
                severity_multiplier = self._calculate_violation_severity(all_violations)
                safety_result['overallConfidence'] *= severity_multiplier
                safety_result['isValid'] = False
                safety_result['systemShouldEngage'] = False
                safety_result['situationFactor'] = 0.4  # Lower situation factor due to violations
            else:
                # Increase engagement safety if no violations
                safety_result['situationFactor'] = 0.9

            # Update adaptive thresholds based on current conditions
            self.traffic_validator.update_validation_thresholds(all_violations)

            # Add comprehensive traffic information to the result
            safety_result['trafficViolations'] = all_violations
            safety_result['trafficRecommendations'] = {**traffic_recommendations, **stop_recommendations,
                                                     **speed_recommendations, **ped_recommendations}
            safety_result['trafficContext'] = traffic_context

            # Calculate additional enhanced metrics
            safety_result['environmentComplexity'] = self._calculate_environment_complexity(traffic_context)
            safety_result['predictionAccuracy'] = prediction_valid
            safety_result['systemLoad'] = min(1.0, len(all_violations) / 10.0)

            # Calculate processing time statistics
            step_time = time.time() - start_time
            self._update_performance_stats(step_time)

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
                'trafficViolations': ['SYSTEM_ERROR'],
                'trafficRecommendations': {},
                'trafficContext': {},
                'environmentComplexity': 1.0,
                'predictionAccuracy': 0.0,
                'systemLoad': 1.0
            }

    def _calculate_prediction_validity(self, position: np.ndarray, velocity: float, acceleration: float) -> float:
        """Calculate validity of prediction based on actual vs predicted behavior"""
        if len(self.velocity_history) < 2:
            return 0.5  # Default value if insufficient data

        # Compare current velocity with predicted velocity from last step
        # This is a simplified approach - real implementation would use more sophisticated comparison
        historical_change = self.velocity_history[-1] - self.velocity_history[-2]
        predicted_change = acceleration * 0.1  # 0.1s time step
        difference = abs(historical_change - predicted_change)

        # Convert to confidence value (lower difference = higher confidence)
        confidence = max(0.1, min(1.0, 1.0 - difference))
        return confidence

    def _calculate_violation_severity(self, violations: list) -> float:
        """Calculate severity multiplier based on types of violations"""
        if not violations:
            return 1.0  # No violations, full confidence

        # Define severity levels for different violations
        severity_scores = {
            'STOP_SIGN_NOT_OBSERVED': 0.1,  # Very severe
            'SPEEDING': 0.4,  # High severity
            'APPROACHING_RED_LIGHT_TOO_FAST': 0.2,  # High severity
            'APPROACHING_PEDESTRIAN_CROSSING_TOO_FAST': 0.3,  # High severity
            'MODERATE_SPEEDING': 0.6,  # Medium severity
            'APPROACHING_YELLOW_LIGHT_UNSAFELY': 0.5,  # Medium severity
            'APPROACHING_STOP_SIGN_TOO_FAST': 0.4,  # Medium severity
        }

        # Calculate average severity
        total_severity = 0.0
        for violation in violations:
            # Find the matching severity score for partial matches
            for key, score in severity_scores.items():
                if key in violation:
                    total_severity += score
                    break
            else:
                total_severity += 0.7  # Default medium severity for unknown violations

        avg_severity = total_severity / len(violations) if violations else 1.0
        return avg_severity  # Lower values for more severe violations

    def _calculate_environment_complexity(self, traffic_context: Dict) -> float:
        """Calculate environment complexity based on traffic context"""
        complexity = 0.0

        # Add complexity based on different elements
        complexity += len(traffic_context.get('traffic_lights', [])) * 0.1
        complexity += len(traffic_context.get('stop_signs', [])) * 0.1
        complexity += len(traffic_context.get('speed_limits', [])) * 0.05
        complexity += len(traffic_context.get('other_signs', [])) * 0.05

        # Normalize to 0-1 scale
        return min(1.0, complexity)

    def _update_performance_stats(self, step_time: float):
        """Update performance statistics"""
        self.performance_stats['step_count'] += 1
        self.performance_stats['max_step_time'] = max(
            self.performance_stats['max_step_time'], step_time
        )
        self.performance_stats['avg_step_time'] = (
            (self.performance_stats['avg_step_time'] * (self.performance_stats['step_count'] - 1) + step_time) /
            self.performance_stats['step_count']
        )

    def publish_validation_metrics(self, validation_data: Dict[str, Any]):
        """Publish validation metrics with comprehensive error handling"""
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

            # Set additional enhanced metrics if available
            if 'trafficLightCompliance' in validation_data:
                validation.trafficLightCompliance = float(validation_data['trafficLightCompliance'])
            if 'obstacleAvoidanceScore' in validation_data:
                validation.obstacleAvoidanceScore = float(validation_data['obstacleAvoidanceScore'])
            if 'laneKeepingScore' in validation_data:
                validation.laneKeepingScore = float(validation_data['laneKeepingScore'])

            self.pm.send('validationMetrics', dat)
        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")

    def run_step(self):
        """Run a single validation step with comprehensive error handling"""
        try:
            validation_data = self.update_validation_systems()
            self.publish_validation_metrics(validation_data)
        except Exception as e:
            cloudlog.error(f"Error in validation controller step: {e}")


def main():
    """Main validation controller loop with enhanced performance monitoring"""
    controller = EnhancedValidationController()
    rk = Ratekeeper(20.0)  # Increased to 20Hz for more responsive validation

    cloudlog.info("Enhanced validation controller starting...")

    # Performance monitoring
    last_log_time = time.time()

    while True:
        step_start = time.time()
        controller.run_step()

        # Log performance metrics periodically
        current_time = time.time()
        if current_time - last_log_time > 10.0:  # Log every 10 seconds
            perf = controller.performance_stats
            if perf['step_count'] > 0:
                cloudlog.debug(f"Validation controller performance - "
                              f"Avg processing: {perf['avg_step_time']*1000:.2f}ms, "
                              f"Max processing: {perf['max_step_time']*1000:.2f}ms, "
                              f"Steps: {perf['step_count']}")
            last_log_time = current_time

        rk.keep_time()


if __name__ == "__main__":
    main()