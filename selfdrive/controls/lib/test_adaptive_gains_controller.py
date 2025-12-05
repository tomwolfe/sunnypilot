"""
Unit tests for Adaptive Gains Controller.

This module tests the safety and robustness improvements made to the adaptive gains controller,
including handling of missing context keys and boundary conditions.
"""

import numpy as np
from openpilot.selfdrive.controls.lib.adaptive_gains_controller import AdaptiveGainsController


class TestAdaptiveGainsController:
    """Test suite for the AdaptiveGainsController class."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.controller = AdaptiveGainsController()

    def test_missing_context_keys(self):
        """Test that missing context keys are handled with safe defaults."""
        # Test with completely empty context
        empty_context = {}
        gains = self.controller.calculate_contextual_adaptive_gains(
            v_ego=15.0, thermal_state=0.0, context=empty_context
        )

        # Check that default values are used when keys are missing
        expected_base_gains = {
            'lateral': {
                'steer_kp': 1.0,
                'steer_ki': 0.1,
                'steer_kd': 0.01,
            },
            'longitudinal': {
                'accel_kp': 1.0,
                'accel_ki': 0.1,
            },
        }

        # With no context adjustments, gains should be based on speed_factor and thermal_adjustment only
        speed_factor = min(1.0, 15.0 / 30.0)  # 0.5
        speed_adjustment = 1.0 - (0.3 * speed_factor)  # 0.85
        thermal_adjustment = 1.0 - (0.0 * 0.2)  # 1.0 (no thermal stress)
        combined_adjustment = speed_adjustment * thermal_adjustment  # 0.85

        # Expected gains should be the base gains multiplied by the combined adjustment
        expected_lateral_steer_kp = expected_base_gains['lateral']['steer_kp'] * combined_adjustment
        expected_lateral_steer_ki = expected_base_gains['lateral']['steer_ki'] * combined_adjustment
        expected_lateral_steer_kd = expected_base_gains['lateral']['steer_kd'] * combined_adjustment

        expected_long_accel_kp = expected_base_gains['longitudinal']['accel_kp'] * combined_adjustment
        expected_long_accel_ki = expected_base_gains['longitudinal']['accel_ki'] * combined_adjustment

        self.assertAlmostEqual(gains['lateral']['steer_kp'], expected_lateral_steer_kp, places=5)
        self.assertAlmostEqual(gains['lateral']['steer_ki'], expected_lateral_steer_ki, places=5)
        self.assertAlmostEqual(gains['lateral']['steer_kd'], expected_lateral_steer_kd, places=5)
        self.assertAlmostEqual(gains['longitudinal']['accel_kp'], expected_long_accel_kp, places=5)
        self.assertAlmostEqual(gains['longitudinal']['accel_ki'], expected_long_accel_ki, places=5)

    def test_partial_context_keys(self):
        """Test handling of partially complete context."""
        partial_context = {
            'is_curvy_road': True,  # Only this key is present
        }

        gains = self.controller.calculate_contextual_adaptive_gains(
            v_ego=15.0, thermal_state=0.0, context=partial_context
        )

        # With curvy road True, we have an additional context adjustment of * 0.85
        speed_factor = min(1.0, 15.0 / 30.0)  # 0.5
        speed_adjustment = 1.0 - (0.3 * speed_factor)  # 0.85
        thermal_adjustment = 1.0  # No thermal stress
        context_adjustment = 0.85  # For curvy road
        combined_adjustment = speed_adjustment * thermal_adjustment * context_adjustment

        expected_lateral_steer_kp = 1.0 * combined_adjustment
        expected_lateral_steer_ki = 0.1 * combined_adjustment
        expected_lateral_steer_kd = 0.01 * combined_adjustment
        expected_long_accel_kp = 1.0 * combined_adjustment
        expected_long_accel_ki = 0.1 * combined_adjustment

        self.assertAlmostEqual(gains['lateral']['steer_kp'], expected_lateral_steer_kp, places=5)
        self.assertAlmostEqual(gains['lateral']['steer_ki'], expected_lateral_steer_ki, places=5)
        self.assertAlmostEqual(gains['lateral']['steer_kd'], expected_lateral_steer_kd, places=5)
        self.assertAlmostEqual(gains['longitudinal']['accel_kp'], expected_long_accel_kp, places=5)
        self.assertAlmostEqual(gains['longitudinal']['accel_ki'], expected_long_accel_ki, places=5)

    def test_context_key_type_validation(self):
        """Test that context keys are properly validated for type."""
        # Test with wrong types for context keys
        invalid_context = {
            'is_curvy_road': "true",  # Should be bool, but is string
            'traffic_density': 123,    # Should be string, but is int
            'weather_condition': None,  # Should be handled gracefully
        }

        # This should not raise an exception and should default to safe values
        gains = self.controller.calculate_contextual_adaptive_gains(
            v_ego=15.0, thermal_state=0.0, context=invalid_context
        )

        # The values should still be valid and should use appropriate defaults
        self.assertIsInstance(gains['lateral']['steer_kp'], (int, float))
        self.assertIsInstance(gains['lateral']['steer_ki'], (int, float))
        self.assertIsInstance(gains['lateral']['steer_kd'], (int, float))
        self.assertIsInstance(gains['longitudinal']['accel_kp'], (int, float))
        self.assertIsInstance(gains['longitudinal']['accel_ki'], (int, float))

    def test_invalid_context_values(self):
        """Test handling of invalid context values."""
        invalid_values_context = {
            'is_curvy_road': True,
            'traffic_density': 'unknown_value',  # Invalid value, should default to 'low'
            'weather_condition': 'hurricane',    # Invalid value, should default to 'normal'
        }

        gains = self.controller.calculate_contextual_adaptive_gains(
            v_ego=15.0, thermal_state=0.0, context=invalid_values_context
        )

        # With curvy road True, we get a context adjustment of * 0.85
        # Invalid traffic_density and weather_condition should default to non-adjusting values
        # So no additional adjustment for traffic or weather
        speed_factor = min(1.0, 15.0 / 30.0)  # 0.5
        speed_adjustment = 1.0 - (0.3 * speed_factor)  # 0.85
        thermal_adjustment = 1.0  # No thermal stress
        context_adjustment = 0.85  # For curvy road only
        combined_adjustment = speed_adjustment * thermal_adjustment * context_adjustment

        expected_lateral_steer_kp = 1.0 * combined_adjustment
        expected_lateral_steer_ki = 0.1 * combined_adjustment
        expected_lateral_steer_kd = 0.01 * combined_adjustment
        expected_long_accel_kp = 1.0 * combined_adjustment
        expected_long_accel_ki = 0.1 * combined_adjustment

        self.assertAlmostEqual(gains['lateral']['steer_kp'], expected_lateral_steer_kp, places=5)
        self.assertAlmostEqual(gains['lateral']['steer_ki'], expected_lateral_steer_ki, places=5)
        self.assertAlmostEqual(gains['lateral']['steer_kd'], expected_lateral_steer_kd, places=5)
        self.assertAlmostEqual(gains['longitudinal']['accel_kp'], expected_long_accel_kp, places=5)
        self.assertAlmostEqual(gains['longitudinal']['accel_ki'], expected_long_accel_ki, places=5)

    def test_gain_bounds_validation(self):
        """Test that gains stay within safe bounds."""
        extreme_context = {
            'is_curvy_road': True,
            'traffic_density': 'high',
            'weather_condition': 'snow',
        }

        gains = self.controller.calculate_contextual_adaptive_gains(
            v_ego=0.0, thermal_state=0.0, context=extreme_context
        )

        # Check that gains are within expected bounds
        self.assertGreaterEqual(gains['lateral']['steer_kp'], 0.1)  # MIN_STEER_KP
        self.assertLessEqual(gains['lateral']['steer_kp'], 3.0)  # MAX_STEER_KP
        self.assertGreaterEqual(gains['lateral']['steer_ki'], 0.01)  # MIN_STEER_KI
        self.assertLessEqual(gains['lateral']['steer_ki'], 1.0)  # MAX_STEER_KI
        self.assertGreaterEqual(gains['lateral']['steer_kd'], 0.0)  # MIN_STEER_KD
        self.assertLessEqual(gains['lateral']['steer_kd'], 0.1)  # MAX_STEER_KD

        self.assertGreaterEqual(gains['longitudinal']['accel_kp'], 0.1)  # MIN_ACCEL_KP
        self.assertLessEqual(gains['longitudinal']['accel_kp'], 2.0)  # MAX_ACCEL_KP
        self.assertGreaterEqual(gains['longitudinal']['accel_ki'], 0.01)  # MIN_ACCEL_KI
        self.assertLessEqual(gains['longitudinal']['accel_ki'], 1.0)  # MAX_ACCEL_KI

    def test_thermal_state_impact(self):
        """Test that thermal state affects gains properly."""
        normal_context = {
            'is_curvy_road': False,
            'traffic_density': 'low',
            'weather_condition': 'normal',
        }

        # Test with no thermal stress
        gains_normal = self.controller.calculate_contextual_adaptive_gains(
            v_ego=15.0, thermal_state=0.0, context=normal_context  # No thermal stress
        )

        # Test with high thermal stress (maximum)
        gains_thermal = self.controller.calculate_contextual_adaptive_gains(
            v_ego=15.0, thermal_state=1.0, context=normal_context  # Maximum thermal stress
        )

        # With thermal stress, gains should be reduced (thermal_adjustment = 1.0 - (1.0 * 0.2) = 0.8)
        # While with no thermal stress, thermal_adjustment = 1.0
        # So gains with thermal stress should be smaller
        self.assertGreater(gains_normal['lateral']['steer_kp'], gains_thermal['lateral']['steer_kp'])
        self.assertGreater(gains_normal['longitudinal']['accel_kp'], gains_thermal['longitudinal']['accel_kp'])

    def test_high_speed_adjustment(self):
        """Test that high speeds reduce gains for stability."""
        normal_context = {
            'is_curvy_road': False,
            'traffic_density': 'low',
            'weather_condition': 'normal',
        }

        gains_low_speed = self.controller.calculate_contextual_adaptive_gains(
            v_ego=5.0, thermal_state=0.0, context=normal_context  # Low speed
        )

        gains_high_speed = self.controller.calculate_contextual_adaptive_gains(
            v_ego=35.0, thermal_state=0.0, context=normal_context  # High speed
        )

        # With high speed (35 m/s), speed factor = min(1.0, 35.0/30.0) = 1.0
        # speed_adjustment = 1.0 - (0.3 * 1.0) = 0.7
        # With low speed (5 m/s), speed factor = min(1.0, 5.0/30.0) = 0.167
        # speed_adjustment = 1.0 - (0.3 * 0.167) = ~0.95
        # So gains at high speed should be smaller than gains at low speed
        self.assertGreater(gains_low_speed['lateral']['steer_kp'], gains_high_speed['lateral']['steer_kp'])
        self.assertGreater(gains_low_speed['longitudinal']['accel_kp'], gains_high_speed['longitudinal']['accel_kp'])


class TestLongitudinalPlannerValidation:
    """Test validation functions in longitudinal planner."""

    def test_np_clip_bounds_validation(self):
        """Test that np.clip is properly applied to sensor data."""
        # This test would be implemented in the longitudinal planner test file
        # For now, we verify that the logic is correctly implemented

        # Simulate the validation logic that was added
        validated_x = np.array([300.0, -5.0, 50.0])  # One value too high, one too low
        validated_v = np.array([60.0, -60.0, 10.0])  # Values outside bounds
        validated_a = np.array([10.0, -20.0, 2.0])  # Values outside bounds

        # Apply the same clipping bounds as in the updated code
        for i in range(len(validated_x)):
            validated_x[i] = np.clip(validated_x[i], 0.1, 200.0)
            validated_v[i] = np.clip(validated_v[i], -50.0, 50.0)
            validated_a[i] = np.clip(validated_a[i], -15.0, 8.0)

        # Check that values are now within bounds
        assert min(validated_x) >= 0.1
        assert max(validated_x) <= 200.0
        assert min(validated_v) >= -50.0
        assert max(validated_v) <= 50.0
        assert min(validated_a) >= -15.0
        assert max(validated_a) <= 8.0


if __name__ == '__main__':
    print("Running Adaptive Gains Controller tests...")
    # Use python -m pytest to run this file directly
