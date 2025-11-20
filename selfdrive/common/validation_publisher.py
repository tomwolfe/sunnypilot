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
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'liveCalibration', 'lateralPlan'])
    self.pm = messaging.PubMaster(['validationMetrics'])

    # Initialize validation data with safe defaults
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
    self.max_previous_states = 10  # Limit to prevent memory issues
    self.publish_counter = 0  # Track publishing frequency
    self.publish_threshold = 5  # Only publish every 5 iterations when stable
    self.last_published_data = {}  # Track last published data to avoid redundant publishing

  def update_validation_metrics(self):
    """Update validation metrics based on current system state with comprehensive error handling"""
    try:
      self.sm.update(0)

      # Initialize values
      lead_confidences = []
      lane_confidences = []
      road_edge_confidences = []

      # Calculate lead confidence metrics
      if self.sm.updated['modelV2']:
        model_data = self.sm['modelV2']

        if hasattr(model_data, 'leadsV3') and model_data.leadsV3:
          for lead in model_data.leadsV3:
            if hasattr(lead, 'confidence'):
              lead_confidences.append(lead.confidence)

        # Calculate lane confidence metrics
        if hasattr(model_data, 'lateralPlanner') and hasattr(model_data.lateralPlanner, 'laneLineProbs'):
          lane_probs = model_data.lateralPlanner.laneLineProbs
          if lane_probs:
            lane_confidences.extend(lane_probs)

        # Calculate road edge confidence if available
        if hasattr(model_data, 'roadEdgeStds') and model_data.roadEdgeStds:
          # Road edge standard deviations - lower is more certain
          road_edge_confs = [max(0.0, min(1.0, 1.0 - std)) for std in model_data.roadEdgeStds]
          road_edge_confidences.extend(road_edge_confs)

      # Calculate averages
      self.validation_data['leadConfidenceAvg'] = sum(lead_confidences) / len(lead_confidences) if lead_confidences else 0.0
      self.validation_data['leadConfidenceMax'] = max(lead_confidences) if lead_confidences else 0.0
      self.validation_data['laneConfidenceAvg'] = sum(lane_confidences) / len(lane_confidences) if lane_confidences else 0.0
      self.validation_data['roadEdgeConfidenceAvg'] = sum(road_edge_confidences) / len(road_edge_confidences) if road_edge_confidences else 0.0

      # Update previous states for temporal consistency calculation
      current_state = dict(self.validation_data)  # Create a copy of current validation data
      if len(self.previous_states) >= self.max_previous_states:
        self.previous_states = self.previous_states[1:]  # Remove oldest state
      self.previous_states.append(current_state)

      # Calculate temporal consistency using utility function
      from selfdrive.common.validation_utils import calculate_temporal_consistency
      self.validation_data['temporalConsistency'] = calculate_temporal_consistency(current_state, self.previous_states)

      # Calculate overall confidence as weighted average
      weight_lead = 0.3
      weight_lane = 0.3
      weight_edge = 0.2
      weight_temporal = 0.2
      self.validation_data['overallConfidence'] = (
        weight_lead * self.validation_data['leadConfidenceAvg'] +
        weight_lane * self.validation_data['laneConfidenceAvg'] +
        weight_edge * self.validation_data['roadEdgeConfidenceAvg'] +
        weight_temporal * self.validation_data['temporalConsistency']
      )

      # Calculate safety score based on multiple factors
      car_speed = 0.0
      if self.sm.updated['carState'] and hasattr(self.sm['carState'], 'vEgo'):
        car_speed = self.sm['carState'].vEgo or 0.0

      # Safety score decreases at higher speeds if confidence is low
      speed_factor = min(1.0, max(0.0, 1.0 - (car_speed / 50.0)))  # Reduce safety at high speeds
      self.validation_data['safetyScore'] = max(0.0, min(1.0, self.validation_data['overallConfidence'] * speed_factor))

      # Update validation status
      conf_thresh = self.validation_data['confidenceThreshold']
      self.validation_data['isValid'] = (
        self.validation_data['overallConfidence'] >= conf_thresh and
        self.validation_data['leadConfidenceAvg'] >= conf_thresh * 0.7 and
        self.validation_data['laneConfidenceAvg'] >= conf_thresh * 0.7
      )

      # System should engage only if validation passes and other conditions are met
      self.validation_data['systemShouldEngage'] = (
        self.validation_data['isValid'] and
        self.validation_data['safetyScore'] >= conf_thresh
      )

    except Exception as e:
      cloudlog.error(f"Error updating validation metrics: {e}")
      # Set safe defaults on error
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

  def should_publish(self) -> bool:
    """Determine if validation metrics should be published based on changes and frequency"""
    # Always publish if system state has changed significantly
    if not self.last_published_data:
      return True

    # Check if any critical values have changed significantly
    critical_keys = ['isValid', 'systemShouldEngage', 'overallConfidence', 'safetyScore']
    for key in critical_keys:
      if key in self.validation_data and key in self.last_published_data:
        if abs(self.validation_data[key] - self.last_published_data[key]) > 0.1:
          return True  # Significant change, should publish

    # If system is stable, use threshold-based publishing
    self.publish_counter = (self.publish_counter + 1) % self.publish_threshold
    return self.publish_counter == 0

  def publish(self):
    """Publish current validation metrics with error handling and optimization"""
    try:
      self.update_validation_metrics()

      # Check if we should publish based on frequency and changes
      if not self.should_publish():
        return  # Skip publishing if not needed

      dat = messaging.new_message('validationMetrics')
      validation = dat.validationMetrics

      # Set validation metrics with error handling
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

      # Update last published data for comparison
      self.last_published_data = dict(self.validation_data)
    except Exception as e:
      cloudlog.error(f"Error publishing validation metrics: {e}")


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