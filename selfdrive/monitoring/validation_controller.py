#!/usr/bin/env python3
"""
Robust Validation Controller for Sunnypilot
Comprehensive validation controller that coordinates all validation systems with error handling
"""
import numpy as np
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from selfdrive.common.performance_monitor import perf_monitor
from sunnypilot.selfdrive.controls.lib.traffic_light_validation import TrafficValidator, TrafficSignData, TrafficSignType
from sunnypilot.selfdrive.controls.lib.traffic_sign_detection import SimpleTrafficSignHandler
from selfdrive.common.enhanced_validation import SimpleValidation


class ValidationController:
  """Robust validation controller that coordinates all validation systems with comprehensive error handling"""

  def __init__(self):
    # Messaging
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'cameraOdometry', 'liveCalibration'])
    self.pm = messaging.PubMaster(['validationMetrics'])

    # Initialize validation components
    self.traffic_validator = TrafficValidator()  # Updated to use improved TrafficValidator
    self.traffic_handler = SimpleTrafficSignHandler()
    self.enhanced_validator = SimpleValidation()

    # State tracking
    self.position_history = []
    self.max_position_history = 20  # Increased for better analysis
    self.velocity_history = []
    self.max_velocity_history = 20

  @perf_monitor.monitor_performance("update_validation_systems")
  def update_validation_systems(self):
    """Update all validation systems with comprehensive error handling"""
    try:
      self.sm.update(0)

      # Process model data for traffic sign detection
      if self.sm.updated['modelV2']:
        model_data = self.sm['modelV2']
        traffic_signs = self.traffic_handler.process_modeld_output(model_data)

        # Add detected traffic signs to validator
        for sign in traffic_signs:
          self.traffic_validator.add_traffic_sign_data(sign)

      # Get car state for validation
      velocity = 0.0
      if self.sm.updated['carState']:
        velocity = self.sm['carState'].vEgo if hasattr(self.sm['carState'], 'vEgo') else 0.0
        # Add to velocity history for temporal analysis
        self.velocity_history.append(velocity)
        if len(self.velocity_history) > self.max_velocity_history:
          self.velocity_history = self.velocity_history[-self.max_velocity_history:]

      # Get position for validation
      position = np.array([0.0, 0.0, 0.0])
      if self.sm.updated['cameraOdometry']:
        odometry = self.sm['cameraOdometry']
        if hasattr(odometry, 'trans'):
          # Extract position from camera odometry
          position = np.array([odometry.trans[0], odometry.trans[1], odometry.trans[2]])

      # Add position to history for temporal validation
      self.position_history.append(position.copy())
      if len(self.position_history) > self.max_position_history:
        self.position_history = self.position_history[-self.max_position_history:]

      # Run traffic light validation
      traffic_safe, traffic_violations = self.traffic_validator.validate_traffic_lights(
        position, velocity
      )

      # Run stop sign validation
      stop_safe, stop_violations = self.traffic_validator.validate_stop_signs(
        position, velocity
      )

      # Get traffic situation assessment
      traffic_assessment = self.traffic_validator.get_traffic_situation_assessment(
        position, velocity
      )

      # Perform enhanced validation with more detailed parameters
      lead_valid, lead_conf = True, 0.8  # Would use real lead data in production
      lane_valid, lane_conf = True, 0.85  # Would use real lane data in production

      # Calculate temporal consistency using velocity history
      temporal_valid = True
      if len(self.velocity_history) > 1:
        temporal_valid = self.enhanced_validator.validate_temporal_consistency(
          velocity, self.velocity_history[-5:] if len(self.velocity_history) > 5 else self.velocity_history
        )

      # Get overall safety score with traffic violations factored in
      safety_result = self.enhanced_validator.get_overall_safety_score(
        lead_valid, lane_valid, temporal_valid, lead_conf, lane_conf
      )

      # Factor in traffic safety violations
      if traffic_violations or stop_violations:
        safety_result['overallConfidence'] *= 0.5  # Significant reduction for traffic violations
        safety_result['isValid'] = False
        safety_result['systemShouldEngage'] = False
        safety_result['situationFactor'] = 0.6  # Lower situation factor due to violations
      elif traffic_assessment['risk_level'] == 'HIGH':
        safety_result['overallConfidence'] *= 0.7  # Reduce confidence in high-risk situations
        safety_result['systemShouldEngage'] = False
      elif traffic_assessment['risk_level'] == 'MEDIUM':
        safety_result['overallConfidence'] *= 0.85  # Reduce confidence in medium-risk situations

      # Add traffic assessment information to the result
      safety_result['trafficViolations'] = traffic_violations + stop_violations
      safety_result['riskLevel'] = traffic_assessment['risk_level']
      safety_result['recommendedAction'] = traffic_assessment['recommended_action']

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
        'riskLevel': 'HIGH',
        'recommendedAction': 'STOP_IMMEDIATELY'
      }

  def publish_validation_metrics(self, validation_data: Dict[str, Any]):
    """Publish validation metrics with error handling"""
    try:
      dat = messaging.new_message('validationMetrics')
      validation = dat.validationMetrics

      # Set validation metrics from the computed data with error handling
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
    """Run a single validation step with error handling"""
    try:
      validation_data = self.update_validation_systems()
      self.publish_validation_metrics(validation_data)
    except Exception as e:
      cloudlog.error(f"Error in validation controller step: {e}")


def main():
  """Main validation controller loop"""
  controller = ValidationController()
  rk = Ratekeeper(10.0)  # 10Hz for responsive validation

  cloudlog.info("Validation controller starting...")

  while True:
    controller.run_step()
    rk.keep_time()


if __name__ == "__main__":
  main()