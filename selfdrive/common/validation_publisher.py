#!/usr/bin/env python3
"""
Validation Publisher for Sunnypilot
Publishes sophisticated validation metrics for safety system
"""
import time
from typing import Dict, Any
from collections import deque

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from selfdrive.common.validation_config import get_validation_config


class ValidationPublisher:
    """Validation publisher for safety metrics with advanced algorithms"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'radarState', 'liveCalibration', 'deviceState'])
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
            'situationFactor': 1.0,  # Multiplier based on driving situation
            'temporalConsistency': 1.0,  # How consistent data is over time
            'systemShouldEngage': False,
            'isValid': False,
            'confidenceThreshold': self.config.confidence_threshold,
        }

        # Initialize state tracking for temporal consistency
        self.lead_confidence_history = deque(maxlen=10)
        self.lane_confidence_history = deque(maxlen=10)
        self.speed_history = deque(maxlen=10)

    def update_validation_metrics(self):
        """Update validation metrics from model and sensor data with sophisticated algorithms"""
        try:
            self.sm.update(0)

            # Process lead data from radar
            if self.sm.updated['radarState']:
                radar_state = self.sm['radarState']
                lead_confidences = []

                # Process all leads, not just leadOne
                if hasattr(radar_state, 'leadOne') and radar_state.leadOne:
                    lead = radar_state.leadOne
                    if lead.status and lead.modelProb > 0:
                        lead_confidences.append(lead.modelProb)

                if hasattr(radar_state, 'leadTwo') and radar_state.leadTwo:
                    lead = radar_state.leadTwo
                    if lead.status and lead.modelProb > 0:
                        lead_confidences.append(lead.modelProb)

                if lead_confidences:
                    self.validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences)
                    self.validation_data['leadConfidenceMax'] = max(lead_confidences)
                    # Add to history for temporal consistency
                    self.lead_confidence_history.extend(lead_confidences)

            # Process model data for lane confidences
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Calculate lane confidences from model
                if (hasattr(model_data, 'lLane') and hasattr(model_data, 'rLane') and
                    hasattr(model_data.lLane, 'proba') and hasattr(model_data.rLane, 'proba')):
                    l_lane_conf = model_data.lLane.proba
                    r_lane_conf = model_data.rLane.proba
                    lane_conf = (l_lane_conf + r_lane_conf) / 2.0
                    self.validation_data['laneConfidenceAvg'] = lane_conf
                    # Add to history for temporal consistency
                    self.lane_confidence_history.append(lane_conf)

                # Calculate road edge confidence
                if hasattr(model_data, 'meta') and hasattr(model_data.meta, 'engageProb'):
                    self.validation_data['roadEdgeConfidenceAvg'] = model_data.meta.engageProb

            # Process car state for speed and situation context
            car_state = self.sm['carState'] if self.sm.updated['carState'] else None
            if car_state and hasattr(car_state, 'vEgo'):
                current_speed = car_state.vEgo
                self.speed_history.append(current_speed)

                # Calculate situation factor based on speed and context
                self.validation_data['situationFactor'] = self._calculate_situation_factor(current_speed)

            # Calculate temporal consistency
            self.validation_data['temporalConsistency'] = self._calculate_temporal_consistency()

            # Calculate overall confidence with weighted factors
            avg_lead = self.validation_data['leadConfidenceAvg']
            avg_lane = self.validation_data['laneConfidenceAvg']
            avg_edge = self.validation_data['roadEdgeConfidenceAvg']
            situation_factor = self.validation_data['situationFactor']
            temporal_consistency = self.validation_data['temporalConsistency']

            # Weighted confidence calculation
            confidence_components = [avg_lead, avg_lane, avg_edge]
            valid_components = [x for x in confidence_components if x > 0]

            if valid_components:
                base_confidence = sum(valid_components) / len(valid_components)
                # Apply situation and temporal factors
                self.validation_data['overallConfidence'] = base_confidence * situation_factor * temporal_consistency
            else:
                self.validation_data['overallConfidence'] = 0.0

            # Calculate safety score with more sophisticated algorithm
            self.validation_data['safetyScore'] = self._calculate_safety_score()

            # Determine if system should engage with additional safety checks
            threshold = self.validation_data['confidenceThreshold']
            vehicle_moving = car_state and hasattr(car_state, 'vEgo') and car_state.vEgo > 1.0  # 1 m/s
            controls_allowed = car_state and hasattr(car_state, 'controlsAllowed') and car_state.controlsAllowed
            device_thermal_ok = True  # Check device thermal status if available

            if self.sm.updated['deviceState']:
                device_state = self.sm['deviceState']
                if hasattr(device_state, 'thermalStatus') and device_state.thermalStatus >= log.DeviceState.ThermalStatus.red:
                    device_thermal_ok = False

            self.validation_data['systemShouldEngage'] = (
                self.validation_data['overallConfidence'] >= threshold and
                vehicle_moving and
                controls_allowed and
                device_thermal_ok
            )

            self.validation_data['isValid'] = self.validation_data['systemShouldEngage']

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
                'temporalConsistency': 1.0,
                'systemShouldEngage': False,
                'isValid': False,
                'confidenceThreshold': self.config.confidence_threshold,
            })

    def _calculate_situation_factor(self, speed: float) -> float:
        """Calculate situation factor based on driving conditions"""
        # Lower factor at very low speeds (parking, traffic jams)
        if speed < 2.0:  # Below 7 km/h
            return 0.8
        # Optimal factor at moderate speeds
        elif 2.0 <= speed <= 25.0:  # 7-90 km/h
            return 1.0
        # Reduced factor at high speeds for increased caution
        elif speed > 25.0:  # Above 90 km/h
            return 0.9
        else:
            return 1.0

    def _calculate_temporal_consistency(self) -> float:
        """Calculate temporal consistency of confidence values"""
        consistency_factors = []

        # Check lead confidence consistency
        if len(self.lead_confidence_history) >= 3:
            lead_values = list(self.lead_confidence_history)
            if len(set(lead_values)) == 1:  # All values are the same
                consistency_factors.append(1.0)
            else:
                # Calculate variance - lower variance means higher consistency
                avg = sum(lead_values) / len(lead_values)
                variance = sum((x - avg) ** 2 for x in lead_values) / len(lead_values)
                consistency = max(0.1, 1.0 - variance)  # Range [0.1, 1.0]
                consistency_factors.append(consistency)

        # Check lane confidence consistency
        if len(self.lane_confidence_history) >= 3:
            lane_values = list(self.lane_confidence_history)
            if len(set(lane_values)) == 1:
                consistency_factors.append(1.0)
            else:
                avg = sum(lane_values) / len(lane_values)
                variance = sum((x - avg) ** 2 for x in lane_values) / len(lane_values)
                consistency = max(0.1, 1.0 - variance)
                consistency_factors.append(consistency)

        if consistency_factors:
            return sum(consistency_factors) / len(consistency_factors)
        else:
            return 1.0

    def _calculate_safety_score(self) -> float:
        """Calculate a comprehensive safety score"""
        # Weight different components
        lead_weight = 0.3
        lane_weight = 0.3
        situation_weight = 0.2
        temporal_weight = 0.2

        avg_lead = self.validation_data['leadConfidenceAvg']
        avg_lane = self.validation_data['laneConfidenceAvg']
        situation_factor = self.validation_data['situationFactor']
        temporal_consistency = self.validation_data['temporalConsistency']

        # Normalize situation factor to 0-1 range for scoring
        normalized_situation = min(1.0, max(0.0, situation_factor))

        safety_score = (
            avg_lead * lead_weight +
            avg_lane * lane_weight +
            normalized_situation * situation_weight +
            temporal_consistency * temporal_weight
        )

        return min(1.0, max(0.0, safety_score))  # Clamp between 0 and 1

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
    config = get_validation_config()
    rk = Ratekeeper(config.validation_frequency)

    cloudlog.info("Validation publisher starting...")

    while True:
        validation_publisher.publish()
        rk.keep_time()


if __name__ == "__main__":
    main()