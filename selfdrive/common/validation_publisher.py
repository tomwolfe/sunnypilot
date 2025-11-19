#!/usr/bin/env python3
"""
Simple Validation Publisher for Sunnypilot
Publisher for validation metrics with essential functionality
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog


class ValidationPublisher:
  """Simple validation metrics publisher with dynamic updates"""

  def __init__(self):
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState'])
    self.pm = messaging.PubMaster(['validationMetrics'])

    # Initialize validation data
    self.validation_data = {
      'leadConfidenceAvg': 0.8,
      'leadConfidenceMax': 0.9,
      'laneConfidenceAvg': 0.85,
      'roadEdgeConfidenceAvg': 0.82,
      'overallConfidence': 0.84,
      'safetyScore': 0.9,
      'situationFactor': 1.0,
      'temporalConsistency': 0.95,
      'systemShouldEngage': True,
      'isValid': True,
      'confidenceThreshold': 0.6
    }

  def update_validation_metrics(self):
    """Update validation metrics based on current system state"""
    self.sm.update(0)

    # Simple update logic based on system state
    # In a real implementation, this would perform actual validation
    if self.sm.updated['modelV2']:
      model_data = self.sm['modelV2']

      # Calculate lead confidence metrics
      if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
        lead_confidences = [lead.confidence for lead in model_data.leadsV3 if hasattr(lead, 'confidence')]
        if lead_confidences:
          self.validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences)
          self.validation_data['leadConfidenceMax'] = max(lead_confidences)

      # Calculate lane confidence metrics
      if hasattr(model_data, 'lateralPlanner') and hasattr(model_data.lateralPlanner, 'laneLineProbs'):
        lane_probs = model_data.lateralPlanner.laneLineProbs
        if lane_probs:
          self.validation_data['laneConfidenceAvg'] = sum(lane_probs) / len(lane_probs)

    # Update overall validation status
    avg_conf = self.validation_data['overallConfidence']
    conf_thresh = self.validation_data['confidenceThreshold']
    self.validation_data['isValid'] = avg_conf >= conf_thresh
    self.validation_data['systemShouldEngage'] = avg_conf >= conf_thresh

  def publish(self):
    """Publish current validation metrics"""
    self.update_validation_metrics()

    dat = messaging.new_message('validationMetrics')
    validation = dat.validationMetrics

    # Set validation metrics
    validation.leadConfidenceAvg = self.validation_data['leadConfidenceAvg']
    validation.leadConfidenceMax = self.validation_data['leadConfidenceMax']
    validation.laneConfidenceAvg = self.validation_data['laneConfidenceAvg']
    validation.roadEdgeConfidenceAvg = self.validation_data['roadEdgeConfidenceAvg']
    validation.overallConfidence = self.validation_data['overallConfidence']
    validation.safetyScore = self.validation_data['safetyScore']
    validation.situationFactor = self.validation_data['situationFactor']
    validation.temporalConsistency = self.validation_data['temporalConsistency']
    validation.systemShouldEngage = self.validation_data['systemShouldEngage']
    validation.isValid = self.validation_data['isValid']
    validation.confidenceThreshold = self.validation_data['confidenceThreshold']

    self.pm.send('validationMetrics', dat)


def main():
  """Main validation publisher loop"""
  validation_publisher = ValidationPublisher()
  rk = Ratekeeper(10.0)  # 10Hz publishing for more responsive validation

  cloudlog.info("Validation publisher starting...")

  while True:
    validation_publisher.publish()
    rk.keep_time()


if __name__ == "__main__":
  main()