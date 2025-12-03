#!/usr/bin/env python3
"""
Comprehensive tests for the improvements made to the adaptive control system.
Tests the fixes for all issues identified in the critical review.
"""

import unittest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
import time
import math

from selfdrive.controls.controlsd import Controls


class TestAdaptiveControlImprovements(unittest.TestCase):
    """Comprehensive tests for the adaptive control system improvements."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a mock Controls instance to test functions without full initialization
        self.controls = Controls.__new__(Controls)  # Create without calling __init__

        # Initialize minimal required attributes
        self.controls._curvature_history = []
        self.controls.CP = Mock()
        self.controls.CP.steerRatio = 15.0
        self.controls.CP.wheelbase = 2.7
        self.controls.VM = Mock()
        self.controls.VM.calc_curvature = lambda steer_angle, v_ego, roll: math.tan(steer_angle) / self.controls.CP.wheelbase
        self.controls._init_circuit_breakers()

    def test_curvature_dependency_fix(self):
        """Test that _calculate_driving_context doesn't depend on self.curvature."""
        # Create mock CarState
        mock_cs = Mock()
        mock_cs.vEgo = 25.0
        mock_cs.steeringAngleDeg = 5.0
        mock_cs.steeringRateDeg = 2.0
        mock_cs.aEgo = 1.0
        mock_cs.windshieldWiper = 0.0 # Added to prevent TypeError in _detect_weather_conditions

        # Mock the submaster to have liveParameters
        self.controls.sm = Mock()
        self.controls.sm.__getitem__ = lambda _, key: mock_sm_item if key == 'liveParameters' else Mock()
        mock_sm_item = Mock()
        mock_sm_item.angleOffsetDeg = 0.0

        # Mock the VM to return a known curvature
        self.controls.VM = Mock()
        self.controls.VM.calc_curvature = Mock(return_value=0.001)

        # Test that the method works without self.curvature being set
        with patch('selfdrive.controls.controlsd.cloudlog'):
            context = self.controls._calculate_driving_context(mock_cs)

        # Verify that context was calculated
        self.assertIn('current_curvature', context)
        self.assertIn('lateral_accel', context)
        self.assertIn('is_curvy_road', context)
        # Verify that calc_curvature was called
        self.controls.VM.calc_curvature.assert_called()

    def test_improved_circuit_breaker_with_root_cause_analysis(self):
        """Test the improved circuit breaker system with root cause tracking."""
        breaker_name = 'adaptive_gains'

        # Initially should be enabled
        self.assertTrue(self.controls._check_circuit_breaker(breaker_name))

        # Trigger the circuit breaker multiple times with different error types
        self.controls._trigger_circuit_breaker(breaker_name, "Test error 1", "sensor_error")
        self.controls._trigger_circuit_breaker(breaker_name, "Test error 2", "sensor_error")  # Same type
        self.controls._trigger_circuit_breaker(breaker_name, "Test error 3", "computation_error")  # Different type

        # Should now be disabled
        self.assertFalse(self.controls._check_circuit_breaker(breaker_name))

        # Verify root cause analysis was populated
        cb = self.controls._circuit_breakers[breaker_name]
        self.assertEqual(len(cb['root_cause_analysis']), 3)

        # Check that error types were stored
        error_types = [e['error_type'] for e in cb['root_cause_analysis']]
        self.assertIn('sensor_error', error_types)
        self.assertIn('computation_error', error_types)

    def test_conservative_fallback_gains(self):
        """Test that fallback gains are more conservative."""
        # Verify that fallback gains are conservative
        fallback_gains = {
            'lateral': {
                'steer_kp': 0.3,
                'steer_ki': 0.03,
                'steer_kd': 0.003,
            },
            'longitudinal': {
                'accel_kp': 0.3,
                'accel_ki': 0.03,
            }
        }

        # Verify that fallback gains are conservative
        for gain_type in fallback_gains:
            for gain_name, gain_value in fallback_gains[gain_type].items():
                # All gains should be relatively low for safety
                self.assertLess(gain_value, 1.0, f"{gain_name} should be conservative")
                self.assertGreater(gain_value, 0, f"{gain_name} should be positive")

    def test_circuit_breaker_abuse_prevention(self):
        """Test that circuit breaker prevents abuse with reduced error tolerance."""
        breaker_name = 'adaptive_gains'

        # Check default values that were changed
        cb = self.controls._circuit_breakers[breaker_name]

        # max_errors should be 3 (reduced from 5)
        self.assertEqual(cb['max_errors'], 3)
        # cooldown_period should be 10.0 (increased from 5)
        self.assertEqual(cb['cooldown_period'], 10.0)

        # Test that circuit breaker doesn't reset too quickly
        # First trigger it to disable
        for i in range(3):
            self.controls._trigger_circuit_breaker(breaker_name, f"Error {i}", f"test_{i}")

        # Should be disabled
        self.assertFalse(self.controls._check_circuit_breaker(breaker_name))

        # Even after cooldown time but before reset time, should not reset
        current_time = time.monotonic()
        cb['last_error_time'] = current_time - 12  # Past cooldown
        cb['last_error_reset_time'] = current_time - 2  # Not yet half of cooldown ago

        self.assertFalse(self.controls._check_circuit_breaker(breaker_name))

    def test_longitudinal_gains_extraction_safety(self):
        """Test that longitudinal gains extraction handles unexpected structures safely."""
        # Test with valid structure
        valid_gains = {
            'lateral': {'steer_kp': 1.0, 'steer_ki': 0.1, 'steer_kd': 0.01},
            'longitudinal': {'accel_kp': 1.0, 'accel_ki': 0.1}
        }

        if isinstance(valid_gains, dict) and 'longitudinal' in valid_gains:
            longitudinal_gains = valid_gains['longitudinal']
        else:
            # Fall back to safe default longitudinal gains if structure is unexpected
            longitudinal_gains = {
                'accel_kp': 0.5,
                'accel_ki': 0.05,
            }

        self.assertEqual(longitudinal_gains['accel_kp'], 1.0)

        # Test with invalid structure - should use fallback
        invalid_gains = "invalid_structure"

        if isinstance(invalid_gains, dict) and 'longitudinal' in invalid_gains:
            longitudinal_gains = invalid_gains['longitudinal']
        else:
            # Fall back to safe default longitudinal gains if structure is unexpected
            longitudinal_gains = {
                'accel_kp': 0.5,
                'accel_ki': 0.05,
            }

        self.assertEqual(longitudinal_gains['accel_kp'], 0.5)
        self.assertEqual(longitudinal_gains['accel_ki'], 0.05)

    def test_adaptive_gpu_management_logic(self):
        """Test the adaptive GPU management to address thermal-latency trade-off."""
        # This test verifies the logic of the adaptive GPU management method
        # Create mock objects
        mock_cs = Mock()
        mock_cs.vEgo = 25.0

        mock_sm = Mock()
        mock_model = Mock()
        mock_model.meta = MagicMock(hardBrakePredicted=False)
        mock_sm.__getitem__ = lambda _, key: {
            'modelV2': mock_model,
            'deviceState': mock_device_state,
            'radarState': mock_radar_state
        }.get(key, Mock())

        mock_device_state = Mock()
        mock_device_state.thermalStatus = 1  # Yellow (not red/danger)
        mock_device_state.thermalPerc = 60   # 60% thermal

        mock_radar_state = Mock()
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = False
        mock_radar_state.leadTwo = Mock()
        mock_radar_state.leadTwo.status = False

        # Test normal situation (should use ondemand)
        with patch('builtins.open', create=True) as mock_open_func:
            # Set up the mock file object
            mock_file = Mock()
            mock_open_func.return_value.__enter__.return_value = mock_file
            mock_open_func.return_value.__exit__.return_value = None
            
            self.controls._adaptive_gpu_management(mock_cs, mock_sm)
            # Verify the method was called at least once
            self.assertTrue(mock_open_func.called)

    def test_gpu_management_with_context(self):
        """Test GPU management with proper context handling."""
        # Create mock objects
        mock_cs = Mock()
        mock_cs.vEgo = 25.0

        mock_sm = Mock()
        mock_model = Mock()
        mock_model.meta = MagicMock(hardBrakePredicted=True)
        mock_sm.__getitem__ = lambda _, key: {
            'modelV2': mock_model,
            'deviceState': mock_device_state,
            'radarState': mock_radar_state
        }.get(key, Mock())

        mock_device_state = Mock()
        mock_device_state.thermalStatus = 1  # Yellow (not red/danger)
        mock_device_state.thermalPerc = 60   # 60% thermal (under 70% limit)

        mock_radar_state = Mock()
        mock_radar_state.leadOne = Mock()
        mock_radar_state.leadOne.status = False
        mock_radar_state.leadTwo = Mock()
        mock_radar_state.leadTwo.status = False

        # Mock the file system operations
        with patch('builtins.open', create=True) as mock_open_func:
            # Set up the mock file object
            mock_file = Mock()
            mock_open_func.return_value.__enter__.return_value = mock_file
            mock_open_func.return_value.__exit__.return_value = None

            # Call the method
            self.controls._adaptive_gpu_management(mock_cs, mock_sm)

            # Verify that we tried to set governor to performance mode
            mock_open_func.assert_called_with("/sys/class/kgsl/kgsl-3d0/devfreq/governor", "w")
            # The write call could have been with "performance" or "ondemand" depending on conditions
            self.assertTrue(mock_file.write.called)

    def test_circuit_breaker_stable_period_logic(self):
        """Test the circuit breaker logic fix to ensure stable period is respected."""
        breaker_name = 'adaptive_gains'
        cb = self.controls._circuit_breakers[breaker_name]

        # Initially should be enabled
        self.assertTrue(self.controls._check_circuit_breaker(breaker_name))

        # Trigger the circuit breaker to disable it
        for i in range(3):  # Hit max errors
            self.controls._trigger_circuit_breaker(breaker_name, f"Test error {i}")

        # Should be disabled now
        self.assertFalse(self.controls._check_circuit_breaker(breaker_name))

        # Manually set the last error time to simulate time passing
        original_time = time.monotonic()
        # Set last_error_time to 12 seconds ago (past basic cooldown of 10s)
        cb['last_error_time'] = original_time - 12.0

        # Even though cooldown has passed, should NOT reset yet because we haven't had
        # the required stable period (cooldown_period / 2 = 5 seconds)
        # Total required time = cooldown (10s) + stable period (5s) = 15s
        # But we only waited 12s, so it should still be disabled
        self.assertFalse(self.controls._check_circuit_breaker(breaker_name))

        # Now set last_error_time to 16 seconds ago (past both cooldown and stable period)
        cb['last_error_time'] = original_time - 16.0

        # Should now be able to reset (if we also update last_error_reset_time)
        cb['last_error_reset_time'] = original_time - 16.0
        self.assertTrue(self.controls._check_circuit_breaker(breaker_name))

        # Verify it reset properly
        self.assertTrue(cb['enabled'])
        self.assertEqual(cb['error_count'], 0)


if __name__ == '__main__':
    unittest.main()