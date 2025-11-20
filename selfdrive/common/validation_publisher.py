#!/usr/bin/env python3
"""
Simplified Validation Publisher for Sunnypilot
Publishes essential validation metrics for safety system
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from selfdrive.common.validation_config import get_validation_config


class ValidationPublisher:
    """Simplified validation publisher for safety metrics"""

    def __init__(self):
        self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'radarState'])
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
            'systemShouldEngage': False,
            'isValid': False,
            'confidenceThreshold': self.config.confidence_threshold,
        }

    def update_validation_metrics(self):
        """Update validation metrics from model and sensor data"""
        try:
            self.sm.update(0)

            # Process lead data from radar
            if self.sm.updated['radarState']:
                radar_state = self.sm['radarState']
                if hasattr(radar_state, 'leadOne') and radar_state.leadOne:
                    lead = radar_state.leadOne
                    if lead.status:
                        self.validation_data['leadConfidenceAvg'] = lead.modelProb
                        self.validation_data['leadConfidenceMax'] = lead.modelProb

            # Process model data for lane confidences
            if self.sm.updated['modelV2']:
                model_data = self.sm['modelV2']

                # Calculate lane confidences from model
                if hasattr(model_data, 'l_lane') and hasattr(model_data, 'r_lane'):
                    l_lane = model_data.l_lane
                    r_lane = model_data.r_lane
                    lane_conf = (l_lane.prob + r_lane.prob) / 2.0
                    self.validation_data['laneConfidenceAvg'] = lane_conf

                # Calculate road edge confidence
                if hasattr(model_data, 'meta'):
                    meta = model_data.meta
                    if hasattr(meta, 'engageProb'):
                        self.validation_data['roadEdgeConfidenceAvg'] = meta.engageProb

            # Calculate overall confidence
            avg_lead = self.validation_data['leadConfidenceAvg']
            avg_lane = self.validation_data['laneConfidenceAvg']
            avg_edge = self.validation_data['roadEdgeConfidenceAvg']

            # Simple average for overall confidence
            self.validation_data['overallConfidence'] = (avg_lead + avg_lane + avg_edge) / 3.0

            # Calculate safety score based on overall confidence
            self.validation_data['safetyScore'] = self.validation_data['overallConfidence']

            # Determine if system should engage
            threshold = self.validation_data['confidenceThreshold']
            car_state = self.sm['carState'] if self.sm.updated['carState'] else None
            vehicle_moving = car_state and hasattr(car_state, 'vEgo') and car_state.vEgo > 1.0  # 1 m/s

            self.validation_data['systemShouldEngage'] = (
                self.validation_data['overallConfidence'] >= threshold and
                vehicle_moving
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
                'systemShouldEngage': False,
                'isValid': False,
                'confidenceThreshold': self.config.confidence_threshold,
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