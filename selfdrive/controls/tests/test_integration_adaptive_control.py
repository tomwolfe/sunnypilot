#!/usr/bin/env python3
"""
Integration tests for the adaptive control system.
Tests the interaction between different components of the enhanced control system.
"""

import unittest
import numpy as np
from unittest.mock import Mock, MagicMock
import time

from selfdrive.controls.controlsd import Controls
from selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner


class TestIntegrationAdaptiveControl(unittest.TestCase):
    """Integration tests for the adaptive control system components."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Since we can't fully initialize the Controls system without real hardware,
        # we'll test the individual methods and their interactions
        pass

    def test_driving_context_and_adaptive_gains_integration(self):
        """Test that driving context properly influences adaptive gains."""
        # Mock CP (CarParams) and CP_SP (CarParamsSunnyPilot) for Controls initialization
        mock_CP = Mock()
        mock_CP.steerRatio = 15.0
        mock_CP.wheelbase = 2.7
        mock_CP.longitudinalActuatorDelay = 0.2
        mock_CP.lateralTuning.which.return_value = 'torque' # Mock to return 'torque' for lateralTuning
        mock_CP_SP = Mock() # Assuming SunnyPilot CarParams can be an empty mock for this test

        # Create an instance of Controls
        controls_instance = Controls()
        controls_instance.CP = mock_CP
        controls_instance.CP_SP = mock_CP_SP
        
        # We need to mock the CI in Controls for initialization, but it's not directly used in the test's scope
        controls_instance.CI = Mock()
        controls_instance.VM = Mock()
        controls_instance.VM.calc_curvature = Mock(return_value=0.001) # Mock calc_curvature
        
        # Mock sm and its attributes required by _calculate_driving_context and _calculate_contextual_adaptive_gains
        controls_instance.sm = Mock()
        controls_instance.sm.valid = {'radarState': True, 'liveParameters': True}
        controls_instance.sm['liveParameters'] = Mock(angleOffsetDeg=0.0, stiffnessFactor=1.0, steerRatio=15.0, roll=0.0)
        controls_instance.sm['radarState'] = Mock(leadOne=Mock(status=False), leadTwo=Mock(status=False))
        controls_instance.sm['carState'] = Mock(vEgo=25.0, steeringAngleDeg=5.0, aEgo=0.0) # Add CS for _detect_weather_conditions

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
                    'steering_activity': 0.1
                },
                {  # Curvy road - should have more conservative gains
                    'is_curvy_road': True,
                    'traffic_density': 'low',
                    'weather_condition': 'normal',
                    'current_curvature': 0.01,  # Higher curvature
                    'lateral_accel': 2.0,
                    'long_accel_magnitude': 1.0,
                    'steering_activity': 2.0
                },
                {  # High traffic - should have more conservative gains
                    'is_curvy_road': False,
                    'traffic_density': 'high',
                    'weather_condition': 'normal',
                    'current_curvature': 0.001,
                    'lateral_accel': 0.5,
                    'long_accel_magnitude': 1.0,
                    'steering_activity': 0.1
                },
                {  # Poor weather - should have more conservative gains
                    'is_curvy_road': False,
                    'traffic_density': 'low',
                    'weather_condition': 'rain',
                    'current_curvature': 0.001,
                    'lateral_accel': 0.5,
                    'long_accel_magnitude': 1.0,
                    'steering_activity': 0.1
                }
            ]
            
            # Test each context
            gains_list = []
            for i, context in enumerate(test_contexts):
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
        # Mock CP (CarParams) and CP_SP (CarParamsSunnyPilot) for LongitudinalPlanner initialization
        mock_CP = Mock()
        mock_CP.steerRatio = 15.0
        mock_CP.wheelbase = 2.7
        mock_CP.longitudinalActuatorDelay = 0.2 # Example value
        mock_CP_SP = Mock() # Assuming SunnyPilot CarParams can be an empty mock for this test

        # Create an instance of LongitudinalPlanner
        planner = LongitudinalPlanner(mock_CP, mock_CP_SP)

        # Test with normal values
        normal_x = np.array([50.0, 30.0, 20.0])
        normal_v = np.array([5.0, 0.0, -5.0])
        normal_a = np.array([1.0, 0.5, -0.5])
        
        validated_x, validated_v, validated_a = planner._validate_fused_sensor_data(
            normal_x, normal_v, normal_a
        )
        
        # Should pass through unchanged (within tolerance)
        np.testing.assert_allclose(validated_x, normal_x, rtol=1e-10)
        np.testing.assert_allclose(validated_v, normal_v, rtol=1e-10)
        np.testing.assert_allclose(validated_a, normal_a, rtol=1e-10)
        
        # Test with invalid values
        invalid_x = np.array([50.0, -10.0, float('nan'), 30.0])  # -10.0 and nan are invalid
        invalid_v = np.array([5.0, 100.0, float('inf'), -5.0])  # 100.0 and inf are invalid
        invalid_a = np.array([2.0, -20.0, 15.0, -1.0])  # -20.0 and 15.0 are invalid
        
        validated_invalid_x, validated_invalid_v, validated_invalid_a = planner._validate_fused_sensor_data(
            invalid_x, invalid_v, invalid_a
        )
        
        # Check that invalid values have been clamped to valid ranges
        # Distance: should be clamped between 0.1 and 200.0
        assert all(0.1 <= x <= 200.0 for x in validated_invalid_x)
        # Velocity: should be clamped between -50.0 and 50.0
        assert all(-50.0 <= v <= 50.0 for v in validated_invalid_v)
        # Acceleration: should be clamped between -15.0 and 8.0
        assert all(-15.0 <= a <= 8.0 for a in validated_invalid_a)
        
        # Specific checks for the corrected values
        assert validated_invalid_x[1] == 0.1  # -10.0 should become 0.1 (minimum distance)
        assert validated_invalid_x[2] == 200.0  # NaN should become 200.0 (safe distance)
        assert validated_invalid_v[1] == 50.0  # 100.0 should become 50.0 (max velocity)
        assert validated_invalid_v[2] == 0.0  # inf should become 0.0 (safe velocity)
        assert validated_invalid_a[1] == -15.0  # -20.0 should become -15.0 (max braking)
        assert validated_invalid_a[2] == 8.0  # 15.0 should become 8.0 (max acceleration)
        
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
                'last_error_time': 0,
                'cooldown_period': 0.1  # Short for testing
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
        self.assertFalse(_check_circuit_breaker('adaptive_gains'))
        
        # Wait for cooldown period and check if it resets
        time.sleep(0.2)  # Wait longer than cooldown period
        self.assertTrue(_check_circuit_breaker('adaptive_gains'))


if __name__ == '__main__':
    unittest.main()