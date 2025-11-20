#!/usr/bin/env python3
"""
Enhanced Validation Publisher for Sunnypilot
Publisher for validation metrics with sophisticated analysis and performance optimization
"""
import time
from typing import Dict, Any
import math

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from selfdrive.perception.behavior_prediction import EnhancedBehaviorPredictor, ObjectBehaviorType


class EnhancedValidationPublisher:
    """Enhanced validation publisher with sophisticated metrics and adaptive logic"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration', 'lateralPlan',
                                     'radarState', 'liveTracks', 'driverMonitoring', 'longitudinalPlan'])
        self.pm = messaging.PubMaster(['validationMetrics'])

        # Initialize validation data with extended metrics
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
            'confidenceThreshold': 0.6,
            'trafficLightCompliance': 1.0,
            'obstacleAvoidanceScore': 1.0,
            'laneKeepingScore': 1.0,
            'predictionAccuracy': 0.0,
            'environmentComplexity': 0.5
        }

        self.previous_states = []  # Store previous states for temporal consistency
        self.max_previous_states = 20  # Increased for better temporal analysis
        self.publish_counter = 0
        self.publish_threshold = 2  # Publish more frequently for responsiveness
        self.last_published_data = {}

        # Initialize behavior predictor for advanced analysis
        self.behavior_predictor = EnhancedBehaviorPredictor(
            prediction_horizon=3.0,  # 3-second horizon for responsiveness
            dt=0.1  # 10 Hz prediction steps
        )

        # Adaptive thresholds that change based on environment
        self.adaptive_thresholds = {
            'confidence': 0.6,
            'speed_factor': 1.0,
            'weather_factor': 1.0,
            'traffic_factor': 1.0
        }

        # Track performance metrics
        self.performance_metrics = {
            'avg_processing_time': 0.0,
            'max_processing_time': 0.0,
            'processing_time_samples': 0
        }

    def update_validation_metrics(self):
        """Update validation metrics with sophisticated analysis and performance optimization"""
        start_time = time.time()

        try:
            self.sm.update(0)

            # Initialize values
            lead_confidences = []
            lane_confidences = []
            road_edge_confidences = []

            # Calculate lead confidence metrics with enhanced analysis
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
                    for lead in model_data.leadsV3:
                        if hasattr(lead, 'confidence'):
                            lead_confidences.append(lead.confidence)

                # Calculate lane confidence metrics with additional checks
                if hasattr(model_data, 'lateralPlanner') and hasattr(model_data.lateralPlanner, 'laneLineProbs'):
                    lane_probs = model_data.lateralPlanner.laneLineProbs
                    if lane_probs:
                        lane_confidences.extend(lane_probs)

                # Calculate road edge confidence with additional data
                if hasattr(model_data, 'roadEdgeStds') and model_data.roadEdgeStds:
                    road_edge_confs = [max(0.0, min(1.0, 1.0 - std)) for std in model_data.roadEdgeStds]
                    road_edge_confidences.extend(road_edge_confs)

                # Calculate prediction accuracy based on model consistency
                if hasattr(model_data, 'positions') and hasattr(model_data, 'velocities'):
                    self.validation_data['predictionAccuracy'] = self._calculate_prediction_accuracy(model_data)

            # Calculate additional metrics if radar data is available
            if self.sm.updated['radarState']:
                radar_data = self.sm['radarState']
                self.validation_data['environmentComplexity'] = self._calculate_environment_complexity(radar_data)

            # Calculate lane keeping score
            if self.sm.updated['lateralPlan']:
                lateral_data = self.sm['lateralPlan']
                self.validation_data['laneKeepingScore'] = self._calculate_lane_keeping_score(lateral_data)

            # Calculate traffic light compliance based on recent traffic light validation
            if self.sm.updated['carState']:
                car_state = self.sm['carState']
                self.validation_data['trafficLightCompliance'] = self._calculate_traffic_compliance(car_state)

            # Calculate averages
            self.validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences) if lead_confidences else 0.0
            self.validation_data['leadConfidenceMax'] = max(lead_confidences) if lead_confidences else 0.0
            self.validation_data['laneConfidenceAvg'] = sum(lane_confidences) / len(lane_confidences) if lane_confidences else 0.0
            self.validation_data['roadEdgeConfidenceAvg'] = sum(road_edge_confidences) / len(road_edge_confidences) if road_edge_confidences else 0.0

            # Update previous states for temporal consistency calculation
            current_state = dict(self.validation_data)
            if len(self.previous_states) >= self.max_previous_states:
                self.previous_states = self.previous_states[1:]
            self.previous_states.append(current_state)

            # Calculate temporal consistency
            self.validation_data['temporalConsistency'] = self._calculate_enhanced_temporal_consistency(
                current_state, self.previous_states
            )

            # Calculate overall confidence with enhanced weighting
            weights = {
                'lead': 0.25,
                'lane': 0.25,
                'edge': 0.15,
                'temporal': 0.15,
                'prediction': 0.1,
                'environment': 0.1
            }
            self.validation_data['overallConfidence'] = (
                weights['lead'] * self.validation_data['leadConfidenceAvg'] +
                weights['lane'] * self.validation_data['laneConfidenceAvg'] +
                weights['edge'] * self.validation_data['roadEdgeConfidenceAvg'] +
                weights['temporal'] * self.validation_data['temporalConsistency'] +
                weights['prediction'] * self.validation_data['predictionAccuracy'] +
                weights['environment'] * (1.0 - self.validation_data['environmentComplexity'])
            )

            # Calculate safety score with adaptive factors
            car_speed = 0.0
            if self.sm.updated['carState'] and hasattr(self.sm['carState'], 'vEgo'):
                car_speed = self.sm['carState'].vEgo or 0.0

            # Apply adaptive thresholds based on environment
            self._update_adaptive_thresholds(car_speed)

            # Apply safety factors
            speed_factor = self.adaptive_thresholds['speed_factor']
            self.validation_data['safetyScore'] = max(0.0, min(1.0,
                self.validation_data['overallConfidence'] *
                speed_factor *
                self.adaptive_thresholds['weather_factor'] *
                self.adaptive_thresholds['traffic_factor']
            ))

            # Update validation status with enhanced criteria
            conf_thresh = self.adaptive_thresholds['confidence']
            self.validation_data['isValid'] = (
                self.validation_data['overallConfidence'] >= conf_thresh and
                self.validation_data['leadConfidenceAvg'] >= conf_thresh * 0.7 and
                self.validation_data['laneConfidenceAvg'] >= conf_thresh * 0.7 and
                self.validation_data['safetyScore'] >= conf_thresh * 0.8  # Higher safety threshold
            )

            # System should engage only if validation passes with all conditions
            self.validation_data['systemShouldEngage'] = (
                self.validation_data['isValid'] and
                self.validation_data['safetyScore'] >= conf_thresh and
                self.validation_data['trafficLightCompliance'] >= 0.5 and
                self.validation_data['laneKeepingScore'] >= 0.5
            )

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
                'confidenceThreshold': 0.6,
                'trafficLightCompliance': 0.0,
                'obstacleAvoidanceScore': 0.0,
                'laneKeepingScore': 0.0,
                'predictionAccuracy': 0.0,
                'environmentComplexity': 1.0
            })

        # Update performance metrics
        processing_time = time.time() - start_time
        self.performance_metrics['processing_time_samples'] += 1
        self.performance_metrics['max_processing_time'] = max(
            self.performance_metrics['max_processing_time'], processing_time
        )
        self.performance_metrics['avg_processing_time'] = (
            (self.performance_metrics['avg_processing_time'] * (self.performance_metrics['processing_time_samples'] - 1) + processing_time) /
            self.performance_metrics['processing_time_samples']
        )

    def _calculate_enhanced_temporal_consistency(self, current_state: Dict, previous_states: list) -> float:
        """Calculate advanced temporal consistency with multiple factors"""
        if len(previous_states) < 3:
            return 1.0 if previous_states else 0.0

        # Calculate consistency across multiple metrics
        consistency_scores = []
        metrics_to_check = ['leadConfidenceAvg', 'laneConfidenceAvg', 'overallConfidence']

        for metric in metrics_to_check:
            if metric in current_state:
                current_value = current_state[metric]
                # Calculate variance from recent values
                recent_values = [state.get(metric, current_value) for state in previous_states[-5:]]
                if len(recent_values) > 1:
                    variance = sum((v - current_value) ** 2 for v in recent_values) / len(recent_values)
                    # Convert variance to consistency score (higher for lower variance)
                    consistency = 1.0 / (1.0 + variance * 10)  # Scale factor for appropriate range
                    consistency_scores.append(consistency)
                else:
                    consistency_scores.append(1.0)

        # Return average consistency, minimum 0.1
        return max(0.1, sum(consistency_scores) / len(consistency_scores)) if consistency_scores else 1.0

    def _calculate_prediction_accuracy(self, model_data) -> float:
        """Calculate prediction accuracy based on model consistency"""
        # For now, return a simple consistency score
        # This would use actual model prediction vs actual outcomes in a full implementation
        return min(1.0, 0.3 + 0.7 * self.validation_data['temporalConsistency'])

    def _calculate_environment_complexity(self, radar_data) -> float:
        """Calculate environment complexity from radar data"""
        if not hasattr(radar_data, 'tracks') or not radar_data.tracks:
            return 0.3  # Low complexity if no track data

        # More tracks means higher complexity
        complexity = min(1.0, len(radar_data.tracks) / 20.0)  # Cap at 20 tracks
        return complexity

    def _calculate_lane_keeping_score(self, lateral_data) -> float:
        """Calculate lane keeping score based on lateral position"""
        if not hasattr(lateral_data, 'dPath') or lateral_data.dPath is None:
            return 0.5

        # Deviation from center line - closer to center is better
        # This is a simplified implementation; real implementation would use actual lane analysis
        return max(0.1, min(1.0, 1.0 - abs(lateral_data.dPath)))

    def _calculate_traffic_compliance(self, car_state) -> float:
        """Calculate traffic compliance based on car state"""
        # Return a basic compliance score based on current compliance status
        # This would be enhanced with actual traffic light/sign data in a complete implementation
        return 1.0  # For now, assume full compliance

    def _update_adaptive_thresholds(self, current_speed: float):
        """Update thresholds based on current driving conditions"""
        # Lower thresholds at higher speeds for safety
        if current_speed > 30.0:  # Over ~108 km/h
            self.adaptive_thresholds['speed_factor'] = 0.7
            self.adaptive_thresholds['confidence'] = 0.7  # Require higher confidence
        elif current_speed > 15.0:  # Over ~54 km/h
            self.adaptive_thresholds['speed_factor'] = 0.85
            self.adaptive_thresholds['confidence'] = 0.65
        else:
            self.adaptive_thresholds['speed_factor'] = 1.0
            self.adaptive_thresholds['confidence'] = 0.6

    def should_publish(self) -> bool:
        """Enhanced logic for determining if metrics should be published"""
        if not self.last_published_data:
            return True

        # Check for significant changes in critical metrics
        critical_keys = ['isValid', 'systemShouldEngage', 'overallConfidence', 'safetyScore']
        for key in critical_keys:
            if key in self.validation_data and key in self.last_published_data:
                old_val = self.last_published_data[key]
                new_val = self.validation_data[key]
                if isinstance(new_val, bool) and new_val != old_val:
                    return True
                elif isinstance(new_val, (int, float)) and abs(new_val - old_val) > 0.1:
                    return True

        # Always publish if safety is compromised
        if not self.validation_data['isValid'] or not self.validation_data['systemShouldEngage']:
            return True

        # Publish every N cycles for regular updates
        self.publish_counter = (self.publish_counter + 1) % self.publish_threshold
        return self.publish_counter == 0

    def publish(self):
        """Enhanced publishing with adaptive frequency and error handling"""
        try:
            self.update_validation_metrics()

            if not self.should_publish():
                return

            dat = messaging.new_message('validationMetrics')
            validation = dat.validationMetrics

            # Set validation metrics with comprehensive error handling
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

            # Set additional enhanced metrics if available
            if 'trafficLightCompliance' in self.validation_data:
                validation.trafficLightCompliance = float(self.validation_data['trafficLightCompliance'])
            if 'obstacleAvoidanceScore' in self.validation_data:
                validation.obstacleAvoidanceScore = float(self.validation_data['obstacleAvoidanceScore'])
            if 'laneKeepingScore' in self.validation_data:
                validation.laneKeepingScore = float(self.validation_data['laneKeepingScore'])

            self.pm.send('validationMetrics', dat)
            self.last_published_data = dict(self.validation_data)

        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")


def main():
    """Main validation publisher loop with performance monitoring"""
    validation_publisher = EnhancedValidationPublisher()
    rk = Ratekeeper(20.0)  # Increased to 20Hz for more responsive validation

    cloudlog.info("Enhanced validation publisher starting...")

    # Performance monitoring
    last_log_time = time.time()

    while True:
        validation_publisher.publish()

        # Log performance metrics periodically
        current_time = time.time()
        if current_time - last_log_time > 10.0:  # Log every 10 seconds
            perf = validation_publisher.performance_metrics
            cloudlog.debug(f"Validation publisher performance - "
                          f"Avg processing: {perf['avg_processing_time']*1000:.2f}ms, "
                          f"Max processing: {perf['max_processing_time']*1000:.2f}ms")
            last_log_time = current_time

        rk.keep_time()


if __name__ == "__main__":
    main()