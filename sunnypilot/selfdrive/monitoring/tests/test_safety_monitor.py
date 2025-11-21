"""
Unit tests for the SafetyMonitor class
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest
import numpy as np
from unittest.mock import Mock
from sunnypilot.selfdrive.monitoring.safety_monitor import SafetyMonitor


def create_mock_model_v2_msg():
  """Create a mock model v2 message for testing"""
  msg = Mock()
  msg.meta = Mock()
  msg.meta.confidence = 0.9
  msg.lateralPlan = Mock()
  msg.lateralPlan.laneWidth = 3.5
  msg.lateralPlan.dPath = [0.1, 0.15, 0.2, 0.18, 0.12]
  msg.path = Mock()
  msg.path.x = list(range(30))
  msg.path.y = [0.001 * i for i in range(30)]  # Gradual curve
  msg.frameId = 100
  return msg


def create_mock_radar_state_msg():
  """Create a mock radar state message for testing"""
  msg = Mock()
  msg.leadOne = Mock()
  msg.leadOne.status = True
  msg.leadOne.dRel = 50.0
  msg.leadOne.vRel = 0.0
  return msg


def create_mock_car_state_msg():
  """Create a mock car state message for testing"""
  msg = Mock()
  msg.vEgo = 15.0
  msg.aEgo = 0.0
  return msg


def create_mock_car_control_msg():
  """Create a mock car control message for testing"""
  msg = Mock()
  msg.orientationNED = [0.0, 0.0, 0.0]
  return msg


def test_safety_monitor_initialization():
  """Test that SafetyMonitor initializes properly"""
  monitor = SafetyMonitor()
  
  assert monitor.model_confidence_threshold == 0.7
  assert monitor.radar_confidence_threshold == 0.6
  assert monitor.lane_deviation_threshold == 0.8
  assert monitor.overall_safety_score == 1.0
  assert monitor.requires_intervention == False


def test_safety_monitor_normal_conditions():
  """Test safety monitor with normal driving conditions"""
  monitor = SafetyMonitor()
  
  model_v2_msg = create_mock_model_v2_msg()
  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()
  
  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )
  
  # With good conditions, safety score should be relatively high
  assert safety_score > 0.7
  assert requires_intervention == False
  assert safety_report['model_confidence'] > 0.7
  assert safety_report['lighting_condition'] == 'normal'


def test_safety_monitor_low_model_confidence():
  """Test safety monitor with low model confidence"""
  monitor = SafetyMonitor()
  
  # Create model with low confidence
  model_v2_msg = create_mock_model_v2_msg()
  model_v2_msg.meta.confidence = 0.3  # Low confidence
  
  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()
  
  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )
  
  # With low model confidence, overall score should be lower
  assert safety_score < 0.7
  assert safety_report['model_confidence'] < 0.5
  assert safety_report['confidence_degraded'] == True


def test_safety_monitor_curve_anticipation():
  """Test safety monitor's curve anticipation feature"""
  monitor = SafetyMonitor()
  
  # Create model with sharp curve ahead
  model_v2_msg = create_mock_model_v2_msg()
  model_v2_msg.path.y = [0.001 * i for i in range(10)] + [0.05 * i for i in range(20)]  # Sharper curve later
  
  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = Mock()
  car_state_msg.vEgo = 25.0  # Higher speed - more critical for curves
  car_state_msg.aEgo = 0.0
  
  car_control_msg = create_mock_car_control_msg()
  
  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )
  
  # With sharp curve at high speed, curve anticipation should be active
  assert safety_report['curve_anticipation_active'] == True
  assert safety_report['curve_anticipation_score'] > 0.0


def test_safety_monitor_lane_deviation():
  """Test safety monitor's lane deviation detection"""
  monitor = SafetyMonitor()

  model_v2_msg = create_mock_model_v2_msg()
  model_v2_msg.lateralPlan.dPath = [0.5, 0.6, 0.7, 0.9, 1.2]  # Deviating from center

  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )

  # With high lane deviation, safety score should be lower
  assert safety_report['lane_deviation'] > 0.5
  assert safety_score < 0.8


def test_safety_monitor_tunnel_lighting_conditions():
  """Test safety monitor's response to tunnel lighting conditions"""
  monitor = SafetyMonitor()

  # Create model with conditions that might indicate tunnel (high confidence lighting)
  model_v2_msg = create_mock_model_v2_msg()
  # Modify path probs to simulate conditions that might indicate tunnel lighting
  model_v2_msg.path.probs = [0.9]  # High confidence path detection

  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )

  # Verify that environmental conditions are properly detected
  assert 'lighting_condition' in safety_report
  assert 'weather_condition' in safety_report
  assert 'road_condition' in safety_report


def test_safety_monitor_weather_detection_with_imu():
  """Test safety monitor's weather detection based on IMU data"""
  monitor = SafetyMonitor()

  model_v2_msg = create_mock_model_v2_msg()
  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()

  # Create car control message with unusual IMU angles that might indicate weather conditions
  car_control_msg = Mock()
  car_control_msg.orientationNED = [0.15, 0.15, 0.0]  # Slightly unusual angles

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )

  # Should detect environmental conditions based on IMU data
  assert 'lighting_condition' in safety_report
  assert 'weather_condition' in safety_report
  assert 'road_condition' in safety_report


def test_safety_monitor_error_handling():
  """Test safety monitor's error handling with invalid inputs"""
  monitor = SafetyMonitor()

  # Create completely invalid model message to test error handling
  invalid_model_v2_msg = Mock()
  invalid_model_v2_msg.path = Mock()
  invalid_model_v2_msg.path.x = []  # Empty path to cause errors
  invalid_model_v2_msg.path.y = []
  invalid_model_v2_msg.path.probs = []
  invalid_model_v2_msg.lateralPlan = Mock()
  invalid_model_v2_msg.lateralPlan.laneWidth = 3.5
  invalid_model_v2_msg.lateralPlan.dPath = []
  invalid_model_v2_msg.frameId = 0

  invalid_radar_state_msg = Mock()
  invalid_radar_state_msg.leadOne = Mock()
  invalid_radar_state_msg.leadOne.status = False  # No lead detection

  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  safety_score, requires_intervention, safety_report = monitor.update(
    invalid_model_v2_msg, invalid_radar_state_msg, car_state_msg, car_control_msg
  )

  # Even with invalid inputs, should return valid safety report
  assert isinstance(safety_score, float)
  assert isinstance(requires_intervention, bool)
  assert isinstance(safety_report, dict)
  assert 'model_confidence' in safety_report
  assert 'overall_safety_score' in safety_report


def test_safety_monitor_low_speed_considerations():
  """Test safety monitor's behavior at low speeds"""
  monitor = SafetyMonitor()

  model_v2_msg = create_mock_model_v2_msg()
  radar_state_msg = create_mock_radar_state_msg()

  # Create car state with very low speed
  car_state_msg = Mock()
  car_state_msg.vEgo = 2.0  # Very low speed
  car_state_msg.aEgo = 0.0

  car_control_msg = create_mock_car_control_msg()

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )

  # With low speed, should have more conservative behavior if safety is poor
  assert isinstance(safety_score, float)
  assert isinstance(requires_intervention, bool)
  assert safety_score >= 0.0 and safety_score <= 1.0


def test_safety_monitor_missing_live_pose():
  """Test safety monitor behavior when livePose is missing/None"""
  monitor = SafetyMonitor()

  model_v2_msg = create_mock_model_v2_msg()
  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  # Test with None live_pose_msg (missing sensor data)
  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg=None
  )

  # Should handle missing livePose gracefully without errors
  assert isinstance(safety_score, float)
  assert isinstance(requires_intervention, bool)
  assert isinstance(safety_report, dict)
  # Should still return reasonable safety metrics
  assert 0.0 <= safety_score <= 1.0


def test_safety_monitor_radar_failure_with_lead():
  """Test safety monitor behavior when radar detects lead but data is unreliable"""
  monitor = SafetyMonitor()

  model_v2_msg = create_mock_model_v2_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  # Create a radar state with lead present but unreliable status
  radar_state_msg = Mock()
  radar_state_msg.leadOne = Mock()
  radar_state_msg.leadOne.status = True  # Lead detected
  radar_state_msg.leadOne.dRel = 50.0
  radar_state_msg.leadOne.vRel = 100.0  # Unusually high relative velocity (unreliable)

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )

  # Should handle unreliable radar data appropriately
  assert isinstance(safety_score, float)
  assert isinstance(safety_report, dict)
  # Radar confidence should be affected by the unreliable data


def test_safety_monitor_conflicting_sensor_data():
  """Test safety monitor behavior with conflicting sensor data"""
  monitor = SafetyMonitor()

  # Create model with high confidence
  model_v2_msg = create_mock_model_v2_msg()
  model_v2_msg.meta.confidence = 0.9  # High model confidence

  # Create radar with low confidence (conflicting with high camera confidence)
  radar_state_msg = Mock()
  radar_state_msg.leadOne = Mock()
  radar_state_msg.leadOne.status = False  # No lead detected by radar

  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )

  # Should handle conflicting sensor data and adjust safety accordingly
  assert isinstance(safety_score, float)
  assert isinstance(safety_report, dict)
  # With conflicting data, safety score should reflect the uncertainty


def test_safety_monitor_sensor_failure_no_radar_lead_with_model_lead():
  """Test safety monitor behavior when radar fails to detect lead but model suggests lead"""
  monitor = SafetyMonitor()

  # Model indicates a lead ahead
  model_v2_msg = create_mock_model_v2_msg()
  # For modelV2, lead information is typically in modelV2.leads[0].dRel or similar,
  # but create_mock_model_v2_msg doesn't include it. Let's assume high confidence implies lead detection.
  # Or, even better, let's make a mock that clearly indicates a lead.
  # For simplicity, we'll assume the base mock_model_v2_msg implicitly supports a lead detection
  # since it has high confidence.

  # Radar indicates no lead
  radar_state_msg = Mock()
  radar_state_msg.leadOne = Mock()
  radar_state_msg.leadOne.status = False  # No lead detected by radar

  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )

  # Radar confidence should be low due to no lead, affecting overall safety score
  assert safety_report['radar_confidence'] < 0.5
  assert isinstance(safety_score, float)
  assert isinstance(safety_report, dict)
  # The overall safety score should be degraded but not critically low, as model still has high confidence.
  assert safety_score < 1.0
  assert safety_score > 0.3 # Should not be in critical intervention mode


def test_safety_monitor_rapid_threshold_transition():
  """Test safety monitor's response to a rapidly decreasing safety score, crossing thresholds."""
  monitor = SafetyMonitor()

  # Mock messages for a consistent environment
  model_v2_msg = create_mock_model_v2_msg()
  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()

  # Initially, normal conditions, high safety score
  safety_score, requires_intervention, safety_report = monitor.update(
      model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )
  assert safety_score > 0.7  # Normal operation
  assert not requires_intervention
  assert not safety_report['confidence_degraded']

  # Simulate a drop to degraded mode (0.5-0.7)
  model_v2_msg.meta.confidence = 0.55 # Puts it in degraded range
  safety_score, requires_intervention, safety_report = monitor.update(
      model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )
  assert safety_score < 0.7 and safety_score > 0.5 # Degraded mode
  assert not requires_intervention # Should not require intervention yet
  assert safety_report['confidence_degraded']

  # Simulate a further drop to high risk (0.3-0.5)
  model_v2_msg.meta.confidence = 0.35 # Puts it in high risk range
  safety_score, requires_intervention, safety_report = monitor.update(
      model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )
  assert safety_score < 0.5 and safety_score > 0.3 # High risk
  assert safety_report['confidence_degraded']

  # Simulate a critical drop (0.0-0.3)
  model_v2_msg.meta.confidence = 0.1 # Puts it in critical range
  safety_score, requires_intervention, safety_report = monitor.update(
      model_v2_msg, radar_state_msg, car_state_msg, car_control_msg
  )
  assert safety_score < 0.3 # Critical
  assert requires_intervention # Should require intervention
  assert safety_report['confidence_degraded']


def test_safety_monitor_default_environmental_conditions():
  """Test that default environmental conditions are 'normal', 'clear', 'dry' after changes."""
  monitor = SafetyMonitor()

  model_v2_msg = create_mock_model_v2_msg()
  radar_state_msg = create_mock_radar_state_msg()
  car_state_msg = create_mock_car_state_msg()
  car_control_msg = create_mock_car_control_msg()
  live_pose_msg = Mock() # Acknowledge live_pose_msg presence but it won't impact env conditions with current logic

  safety_score, requires_intervention, safety_report = monitor.update(
    model_v2_msg, radar_state_msg, car_state_msg, car_control_msg, live_pose_msg=live_pose_msg
  )

  # Assert that environmental conditions are set to default safe values
  assert safety_report['lighting_condition'] == 'normal'
  assert safety_report['weather_condition'] == 'clear'
  assert safety_report['road_condition'] == 'dry'


if __name__ == "__main__":
  pytest.main([__file__])
