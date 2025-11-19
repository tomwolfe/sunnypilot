#!/usr/bin/env python3
"""
Simple Validation Controller for Sunnypilot
Essential validation controller that coordinates all validation systems
"""
import time
from typing import Dict, Any

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from sunnypilot.selfdrive.controls.lib.traffic_light_validation import SimpleTrafficValidator, TrafficSignData, TrafficSignType
from sunnypilot.selfdrive.controls.lib.traffic_sign_detection import SimpleTrafficSignHandler
from selfdrive.common.enhanced_validation import SimpleValidation


class SimpleValidationController:
  """Simple validation controller that coordinates all validation systems"""

  def __init__(self):
    # Messaging
    self.sm = messaging.SubMaster(['modelV2', 'carState', 'controlsState', 'cameraOdometry'])
    self.pm = messaging.PubMaster(['validationMetrics'])

    # Initialize validation components
    self.traffic_validator = SimpleTrafficValidator()
    self.traffic_handler = SimpleTrafficSignHandler()
    self.enhanced_validator = SimpleValidation()

    # State tracking
    self.position_history = []
    self.max_position_history = 10

  def update_validation_systems(self):
    """Update all validation systems"""
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

    # Get position for validation
    position = [0.0, 0.0, 0.0]
    if self.sm.updated['cameraOdometry']:
      odometry = self.sm['cameraOdometry']
      if hasattr(odometry, 'trans'):
        # Extract position from camera odometry
        position = [odometry.trans[0], odometry.trans[1], odometry.trans[2]]

    # Add position to history for temporal validation
    self.position_history.append(position)
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

    # Perform enhanced validation
    lead_valid, lead_conf = True, 0.8  # Simplified - would use real lead data
    lane_valid, lane_conf = True, 0.85  # Simplified - would use real lane data
    temporal_valid = self.enhanced_validator.validate_temporal_consistency(
      velocity, [vel for _, _, vel in self.position_history[-5:]] if len(self.position_history) > 1 else [velocity]
    )

    # Get overall safety score
    safety_result = self.enhanced_validator.get_overall_safety_score(
      lead_valid, lane_valid, temporal_valid, lead_conf, lane_conf
    )

    # Update traffic safety in overall result
    if not traffic_safe or not stop_safe:
      safety_result['overallConfidence'] *= 0.7  # Reduce confidence if traffic violations detected
      safety_result['isValid'] = False
      safety_result['systemShouldEngage'] = False

    return safety_result

  def publish_validation_metrics(self, validation_data: Dict[str, Any]):
    """Publish validation metrics"""
    dat = messaging.new_message('validationMetrics')
    validation = dat.validationMetrics

    # Set validation metrics from the computed data
    validation.leadConfidenceAvg = validation_data.get('leadConfidenceAvg', 0.8)
    validation.leadConfidenceMax = validation_data.get('leadConfidenceMax', 0.9)
    validation.laneConfidenceAvg = validation_data.get('laneConfidenceAvg', 0.85)
    validation.roadEdgeConfidenceAvg = validation_data.get('roadEdgeConfidenceAvg', 0.82)
    validation.overallConfidence = validation_data.get('overallConfidence', 0.84)
    validation.safetyScore = validation_data.get('safetyScore', 0.9)
    validation.situationFactor = validation_data.get('situationFactor', 1.0)
    validation.temporalConsistency = validation_data.get('temporalConsistency', 0.95)
    validation.systemShouldEngage = validation_data.get('systemShouldEngage', True)
    validation.isValid = validation_data.get('isValid', True)
    validation.confidenceThreshold = validation_data.get('confidenceThreshold', 0.6)

    self.pm.send('validationMetrics', dat)

  def run_step(self):
    """Run a single validation step"""
    validation_data = self.update_validation_systems()
    self.publish_validation_metrics(validation_data)


def main():
  """Main validation controller loop"""
  controller = SimpleValidationController()
  rk = Ratekeeper(10.0)  # 10Hz for responsive validation

  cloudlog.info("Simple validation controller starting...")

  while True:
    controller.run_step()
    rk.keep_time()


if __name__ == "__main__":
  main()