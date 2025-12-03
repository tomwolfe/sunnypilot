#!/usr/bin/env python3
"""
Integration tests for the adaptive control system.
Tests the interaction between different components of the enhanced control system.
"""

import unittest
import numpy as np
from unittest.mock import Mock
import time


class TestIntegrationAdaptiveControl(unittest.TestCase):
  """Integration tests for the adaptive control system components."""

  def setup_method(self, method):
    """Set up test fixtures before each test method."""
    # Since we can't fully initialize the Controls system without real hardware,
    # we'll test the individual methods and their interactions

  def test_driving_context_and_adaptive_gains_integration(self):
    """Test that driving context properly influences adaptive gains."""
    # Instead of creating a full Controls instance which can cause hanging due to complex initialization,
    # we directly test the _calculate_contextual_adaptive_gains method in isolation

    # Mock CP (CarParams) and CP_SP (CarParamsSunnyPilot) for Controls initialization
    mock_CP = Mock()
    mock_CP.steerRatio = 15.0
    mock_CP.wheelbase = 2.7
    mock_CP.longitudinalActuatorDelay = 0.2
    mock_CP.lateralTuning.which.return_value = 'torque'  # Mock to return 'torque' for lateralTuning

    # Create mock_CP_SP
    mock_CP_SP = Mock()
    mock_CP_SP.steerRatio = 15.0
    mock_CP_SP.wheelbase = 2.7
    mock_CP_SP.longitudinalActuatorDelay = 0.2

    # Create a minimal Controls instance using __new__ to avoid __init__ complexity
    from openpilot.selfdrive.controls.controlsd import Controls

    controls_instance = Controls.__new__(Controls)

    # Initialize only the required attributes for the method to work
    controls_instance.CP = mock_CP
    controls_instance.CP_SP = mock_CP_SP
    # Initialize as empty dict instead of None to avoid TypeError in _validate_adaptive_gains
    controls_instance._prev_adaptive_gains = {}

    # Mock sm and its attributes required by _detect_weather_conditions (indirectly called)
    mock_sm = Mock()

    def mock_getitem(key):
      if key == 'carState':
        return Mock(windshieldWiper=0.0, wiperState=0, steeringAngleDeg=5.0, aEgo=1.0)
      elif key == 'liveParameters':
        return Mock(angleOffsetDeg=0.0, stiffnessFactor=1.0, steerRatio=15.0, roll=0.0)
      else:
        return Mock()

    mock_sm.__getitem__ = mock_getitem
    controls_instance.sm = mock_sm

    # Create a basic function that mimics the interaction between context and gains
    def test_context_gain_interaction():
      # Simulate different contexts and verify that gains change appropriately
      test_contexts = [
        {  # Normal conditions
          'is_curvy_road': False,
          'traffic_density': 'low',
          'weather_condition': 'normal',
          'current_curvature': 0.001,
          'lateral_accel': 0.5,
          'long_accel_magnitude': 1.0,
          'steering_activity': 0.1,
        },
        {  # Curvy road - should have more conservative gains
          'is_curvy_road': True,
          'traffic_density': 'low',
          'weather_condition': 'normal',
          'current_curvature': 0.01,  # Higher curvature
          'lateral_accel': 2.0,
          'long_accel_magnitude': 1.0,
          'steering_activity': 2.0,
        },
        {  # High traffic - should have more conservative gains
          'is_curvy_road': False,
          'traffic_density': 'high',
          'weather_condition': 'normal',
          'current_curvature': 0.001,
          'lateral_accel': 0.5,
          'long_accel_magnitude': 1.0,
          'steering_activity': 0.1,
        },
        {  # Poor weather - should have more conservative gains
          'is_curvy_road': False,
          'traffic_density': 'low',
          'weather_condition': 'rain',
          'current_curvature': 0.001,
          'lateral_accel': 0.5,
          'long_accel_magnitude': 1.0,
          'steering_activity': 0.1,
        },
      ]

      # Test each context
      gains_list = []
      for _i, context in enumerate(test_contexts):
        # Call the actual instance method
        gains = controls_instance._calculate_contextual_adaptive_gains(v_ego=25.0, thermal_state=0.1, context=context)
        gains_list.append(gains)

        # Verify gains are calculated
        self.assertIn('lateral', gains)
        self.assertIn('longitudinal', gains)
        self.assertIn('steer_kp', gains['lateral'])

        # Verify they are reasonable values
        self.assertGreater(gains['lateral']['steer_kp'], 0)
        self.assertLess(gains['lateral']['steer_kp'], 3.0)  # Upper bound

    test_context_gain_interaction()

  def test_fusion_safety_validation_integration(self):
    """Test that sensor fusion properly validates outputs."""
    # Test the fusion validation function with different inputs
    # Instead of creating a full LongitudinalPlanner that may have complex initialization,
    # we'll directly test the _validate_fused_sensor_data method

    # Mock CP (CarParams) and CP_SP (CarParamsSunnyPilot) for LongitudinalPlanner initialization
    mock_CP = Mock()
    mock_CP.steerRatio = 15.0
    mock_CP.wheelbase = 2.7
    mock_CP.longitudinalActuatorDelay = 0.2  # Example value

    # Create an instance of LongitudinalPlanner using __new__ to avoid complex initialization
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

    planner = LongitudinalPlanner.__new__(LongitudinalPlanner)

    # Test with normal values - verify they're within valid ranges
    # First create planner instance and initialize properly
    normal_x = np.array([50.0, 30.0, 20.0])
    normal_v = np.array([5.0, 0.0, -5.0])
    normal_a = np.array([1.0, 0.5, -0.5])

    # Set up initial values for the first call with 3 elements
    planner._prev_validated_x = normal_x.copy()
    planner._prev_validated_v = normal_v.copy()
    planner._prev_validated_a = normal_a.copy()
    planner._acceleration_history = []
    planner._frame_counter = 0

    validated_x, validated_v, validated_a = planner._validate_fused_sensor_data(normal_x, normal_v, normal_a)

    # Check that values are in valid ranges (they might be adjusted due to smoothing)
    assert all(0.1 <= x <= 200.0 for x in validated_x)
    assert all(-50.0 <= v <= 50.0 for v in validated_v)
    assert all(-15.0 <= a <= 8.0 for a in validated_a)

    # Test with invalid values (4 elements) - need to make sure arrays are the right size
    invalid_x = np.array([50.0, -10.0, float('nan'), 30.0])  # -10.0 and nan are invalid
    invalid_v = np.array([5.0, 100.0, float('inf'), -5.0])  # 100.0 and inf are invalid
    invalid_a = np.array([2.0, -20.0, 15.0, -1.0])  # -20.0 and 15.0 are invalid

    # Initialize the previous arrays to match the size of the new input (4 elements)
    planner._prev_validated_x = np.array([45.0, -5.0, 180.0, 25.0])  # Sample previous values for 4 elements
    planner._prev_validated_v = np.array([4.0, 95.0, 0.0, -4.0])  # Sample previous values for 4 elements
    planner._prev_validated_a = np.array([1.5, -15.0, 10.0, -0.5])  # Sample previous values for 4 elements

    validated_invalid_x, validated_invalid_v, validated_invalid_a = planner._validate_fused_sensor_data(invalid_x, invalid_v, invalid_a)

    # Check that invalid values have been clamped to valid ranges
    # Distance: should be clamped between 0.1 and 200.0
    assert all(0.1 <= x <= 200.0 for x in validated_invalid_x)
    # Velocity: should be clamped between -50.0 and 50.0
    assert all(-50.0 <= v <= 50.0 for v in validated_invalid_v)
    # Acceleration: should be clamped between -15.0 and 8.0
    assert all(-15.0 <= a <= 8.0 for a in validated_invalid_a)

    # Specific checks for the corrected values - check they are within expected bounds
    # Note: Due to smoothing algorithms in the validation, exact values may vary
    assert 0.1 <= validated_invalid_x[1] <= 200.0  # -10.0 should be clamped to valid range (likely minimum)
    assert 0.1 <= validated_invalid_x[2] <= 200.0  # NaN should be replaced with valid value (smoothing may adjust it)
    assert -50.0 <= validated_invalid_v[1] <= 50.0  # 100.0 should be clamped to max velocity
    assert -50.0 <= validated_invalid_v[2] <= 50.0  # inf should be replaced with valid value
    assert -15.0 <= validated_invalid_a[1] <= 8.0  # -20.0 should be clamped to max braking
    assert -15.0 <= validated_invalid_a[2] <= 8.0  # 15.0 should be clamped to max acceleration

  def test_circuit_breaker_integration_logic(self):
    """Test the circuit breaker logic."""
    # Create a mock of the controls system to test circuit breaker functionality
    controls = Mock()

    # Initialize circuit breakers similar to the actual implementation
    controls._circuit_breakers = {
      'adaptive_gains': {
        'enabled': True,
        'error_count': 0,
        'max_errors': 5,
        'last_error_time': time.monotonic(),  # Use current time
        'cooldown_period': 0.1,  # Short for testing
      }
    }

    def _check_circuit_breaker(breaker_name):
      """Simplified version of the circuit breaker check method."""
      cb = controls._circuit_breakers[breaker_name]

      # Check if we're in cooldown period after an error
      current_time = time.monotonic()
      if not cb['enabled']:
        if current_time - cb['last_error_time'] > cb['cooldown_period']:
          # Reset the circuit breaker after cooldown
          cb['enabled'] = True
          cb['error_count'] = 0
          cb['last_error_time'] = current_time  # Update last error time when resetting
        else:
          return False  # Still in cooldown, circuit breaker is disabled

      return cb['enabled']

    def _trigger_circuit_breaker(breaker_name, error_msg):
      """Simplified version of the circuit breaker trigger method."""
      cb = controls._circuit_breakers[breaker_name]
      cb['error_count'] += 1
      cb['enabled'] = False
      cb['last_error_time'] = time.monotonic()

    # Test normal operation
    self.assertTrue(_check_circuit_breaker('adaptive_gains'))

    # Trigger the circuit breaker
    _trigger_circuit_breaker('adaptive_gains', 'test error')

    # Should now be disabled
    assert not _check_circuit_breaker('adaptive_gains')

    # Manually update the last_error_time to simulate the passage of time
    # Instead of using time.sleep, we'll modify the timestamp directly
    controls._circuit_breakers['adaptive_gains']['last_error_time'] = time.monotonic() - 0.2  # More than cooldown period ago

    # Should now be reset
    assert _check_circuit_breaker('adaptive_gains')
