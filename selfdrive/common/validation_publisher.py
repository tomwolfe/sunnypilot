"""
Enhanced validation metrics publisher for Sunnypilot
Publishes comprehensive validation metrics for use across the system
"""

import time
import numpy as np
from typing import Dict, Any
from cereal import log, custom
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params


class ValidationMetricsPublisher:
    """
    Publisher for enhanced validation metrics that can be used system-wide
    """
    
    def __init__(self):
        self.params = Params()
        
        # Initialize messaging
        try:
            self.pm = messaging.PubMaster(['validationMetrics'])
            self.sm = messaging.SubMaster(['modelV2', 'carState', 'radarState'], ignore_alive=True)
        except Exception as e:
            cloudlog.warning(f"Could not initialize messaging in ValidationMetricsPublisher: {e}")
            self.pm = None
            self.sm = None
        
        self.last_publish_time = time.time()
        self.publish_interval = 0.05  # 20Hz publishing
    
    def publish_metrics(self, enhanced_validation_result: Dict[str, Any],
                       model_output: Dict[str, Any] = None,
                       car_state: log.CarState = None) -> None:
        """
        Publish validation metrics to system messaging
        """
        current_time = time.time()

        # Limit publishing rate
        if current_time - self.last_publish_time < self.publish_interval:
            return

        self.last_publish_time = current_time

        try:
            # Validate that we have required data before creating message
            if not enhanced_validation_result:
                cloudlog.warning("No validation result to publish")
                return

            # Create validation metrics message
            msg = messaging.new_message('validationMetrics')
            validation_metrics = msg.validationMetrics

            # Set standard validation metrics with proper bounds checking
            validation_metrics.leadConfidenceAvg = max(0.0, min(1.0, enhanced_validation_result.get('base_confidence', 0.0)))
            validation_metrics.leadConfidenceMax = max(0.0, min(1.0, enhanced_validation_result.get('situation_adjusted_confidence', 0.0)))
            validation_metrics.laneConfidenceAvg = max(0.0, min(1.0, enhanced_validation_result.get('base_confidence', 0.0)))
            validation_metrics.overallConfidence = max(0.0, min(1.0, enhanced_validation_result.get('situation_adjusted_confidence', 0.0)))
            validation_metrics.isValid = bool(enhanced_validation_result.get('system_safe', False))
            validation_metrics.confidenceThreshold = 0.6  # Standard threshold

            # Set enhanced metrics with bounds checking
            validation_metrics.enhanced.situationFactor = max(0.0, min(2.0, enhanced_validation_result.get('situation_factor', 1.0)))
            validation_metrics.enhanced.speedAdjustedConfidence = max(0.0, min(1.0, enhanced_validation_result.get('speed_adjusted_confidence', 0.0)))
            validation_metrics.enhanced.temporalConsistency = max(0.0, min(1.0, enhanced_validation_result.get('temporal_consistency', 1.0)))
            validation_metrics.enhanced.systemSafe = bool(enhanced_validation_result.get('system_safe', False))

            # Additional metrics for specific use cases
            validation_metrics.enhanced.leadConfidenceOk = bool(enhanced_validation_result.get('lead_confidence_ok', False))
            validation_metrics.enhanced.laneConfidenceOk = bool(enhanced_validation_result.get('lane_confidence_ok', False))
            validation_metrics.enhanced.overallConfidenceOk = bool(enhanced_validation_result.get('overall_confidence_ok', False))
            validation_metrics.enhanced.laneChangeSafe = bool(enhanced_validation_result.get('lane_change_safe', True))
            validation_metrics.enhanced.systemEngagementSafe = bool(enhanced_validation_result.get('system_engagement_safe', True))

            # Publish the message
            if self.pm:
                self.pm.send('validationMetrics', msg)

        except Exception as e:
            cloudlog.error(f"Error publishing validation metrics: {e}")
    
    def create_basic_metrics(self, model_output: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create basic validation metrics from model output if enhanced validation is not available
        """
        # Extract basic metrics from model output
        lead_confidence = model_output.get('lead_confidence_avg', 0.0)
        lane_confidence = model_output.get('lane_confidence_avg', 0.0)
        overall_confidence = model_output.get('overall_confidence', 0.0)
        
        return {
            'base_confidence': overall_confidence,
            'situation_factor': 1.0,
            'situation_adjusted_confidence': overall_confidence,
            'speed_adjusted_confidence': overall_confidence,
            'temporal_consistency': model_output.get('temporal_consistency', 1.0),
            'system_safe': overall_confidence >= 0.6,
            'lead_confidence_ok': lead_confidence >= 0.6,
            'lane_confidence_ok': lane_confidence >= 0.65,
            'overall_confidence_ok': overall_confidence >= 0.6,
            'lane_change_safe': overall_confidence >= 0.7 and lane_confidence >= 0.7,
            'system_engagement_safe': True
        }


# Global instance
validation_metrics_publisher = ValidationMetricsPublisher()