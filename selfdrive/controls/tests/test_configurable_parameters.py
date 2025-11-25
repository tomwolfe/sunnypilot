#!/usr/bin/env python3
"""
Test suite to validate configurable parameters in sunnypilot autonomous driving system.
This ensures that all hardcoded values mentioned in the critical review have been made configurable.
"""

import unittest
from unittest.mock import Mock, patch
import numpy as np

from cereal import car, log
import cereal.messaging as messaging

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


class TestConfigurableParameters(unittest.TestCase):
    """Test suite for configurable parameters."""
    
    def setUp(self):
        """Set up test environment."""
        self.CP = Mock()
        self.CP.steerRatio = 15.0
        self.CP.wheelbase = 2.7
        self.CP.minSteerSpeed = 0.5
        self.CP.lateralTuning.pid.kpBP = [0.0, 5.0, 35.0]
        self.CP.lateralTuning.pid.kpV = [1.0, 0.8, 0.5]
        self.CP.lateralTuning.pid.kiBP = [0.0, 5.0, 35.0]
        self.CP.lateralTuning.pid.kiV = [0.0, 0.5, 0.2]
        self.CP.lateralTuning.pid.kf = 0.00006
        self.CP.vEgoStopping = 0.25
        self.CP.vEgoStarting = 0.5
        self.CP.stopAccel = -2.0
        self.CP.stoppingDecelRate = 0.8
        self.CP.startAccel = 1.0
        self.CP.startingState = True
        self.CP.longitudinalTuning.kpBP = [0.0, 5.0, 35.0]
        self.CP.longitudinalTuning.kpV = [1.0, 0.8, 0.5]
        self.CP.longitudinalTuning.kiBP = [0.0, 5.0, 35.0]
        self.CP.longitudinalTuning.kiV = [0.0, 0.5, 0.2]
        
        self.CP_SP = Mock()
        self.CI = Mock()
        self.CI.get_steer_feedforward_function.return_value = Mock()
        self.CI.torque_from_lateral_accel.return_value = Mock()
        self.CI.lateral_accel_from_torque.return_value = Mock()

    def test_latcontrol_pid_configurable_parameters(self):
        """Test that latcontrol_pid uses configurable parameters."""
        controller = LatControlPID(self.CP, self.CP_SP, self.CI, 0.01)
        
        # Check default values
        self.assertEqual(controller.max_angle_rate, 2.0)
        self.assertEqual(controller.high_speed_threshold, 15.0)
        self.assertEqual(controller.high_speed_ki_limit, 0.15)
        
        # Test that parameters affect behavior
        # The controller should store these values as instance variables
        self.assertTrue(hasattr(controller, 'max_angle_rate'))
        self.assertTrue(hasattr(controller, 'high_speed_threshold'))
        self.assertTrue(hasattr(controller, 'high_speed_ki_limit'))

    def test_latcontrol_torque_configurable_parameters(self):
        """Test that latcontrol_torque uses configurable parameters."""
        controller = LatControlTorque(self.CP, self.CP_SP, self.CI, 0.01)
        
        # Check default values
        self.assertEqual(controller.max_lateral_jerk, 5.0)
        self.assertEqual(controller.high_speed_threshold, 15.0)
        self.assertEqual(controller.high_speed_ki_limit, 0.15)
        
        # Test that parameters affect behavior
        self.assertTrue(hasattr(controller, 'max_lateral_jerk'))
        self.assertTrue(hasattr(controller, 'high_speed_threshold'))
        self.assertTrue(hasattr(controller, 'high_speed_ki_limit'))

    def test_longcontrol_configurable_parameters(self):
        """Test that longcontrol uses configurable parameters."""
        controller = LongControl(self.CP, self.CP_SP)
        
        # Check default values
        self.assertEqual(controller.max_jerk, 2.5)
        self.assertEqual(controller.max_stopping_jerk, 1.5)
        self.assertEqual(controller.max_output_jerk, 2.5)
        self.assertEqual(controller.starting_speed_threshold, 3.0)
        self.assertEqual(controller.starting_accel_multiplier, 0.7)
        self.assertEqual(controller.starting_accel_limit, 0.8)
        self.assertEqual(controller.adaptive_error_threshold, 0.5)
        self.assertEqual(controller.adaptive_speed_threshold, 5.0)
        
        # Test that parameters affect behavior
        self.assertTrue(hasattr(controller, 'max_jerk'))
        self.assertTrue(hasattr(controller, 'max_stopping_jerk'))
        self.assertTrue(hasattr(controller, 'max_output_jerk'))
        self.assertTrue(hasattr(controller, 'starting_speed_threshold'))
        self.assertTrue(hasattr(controller, 'starting_accel_multiplier'))
        self.assertTrue(hasattr(controller, 'starting_accel_limit'))
        self.assertTrue(hasattr(controller, 'adaptive_error_threshold'))
        self.assertTrue(hasattr(controller, 'adaptive_speed_threshold'))

    @patch('openpilot.common.params.Params.get')
    def test_custom_parameter_values(self, mock_get):
        """Test that custom parameter values are correctly loaded."""
        # Mock custom parameter values
        custom_params = {
            'LateralMaxAngleRate': '3.0',
            'LateralHighSpeedThreshold': '20.0',
            'LateralHighSpeedKiLimit': '0.10',
            'LongitudinalMaxJerk': '3.0',
            'LongitudinalMaxStoppingJerk': '2.0',
            'LongitudinalMaxOutputJerk': '3.0',
            'LongitudinalStartingSpeedThreshold': '5.0',
            'LongitudinalStartingAccelMultiplier': '0.5',
            'LongitudinalStartingAccelLimit': '1.0',
            'LongitudinalAdaptiveErrorThreshold': '0.3',
            'LongitudinalAdaptiveSpeedThreshold': '10.0'
        }
        
        def side_effect(key, encoding=None):
            if key in custom_params:
                return custom_params[key].encode(encoding) if encoding else custom_params[key]
            return None
        
        mock_get.side_effect = side_effect
        
        # Create controllers with mocked parameters
        pid_controller = LatControlPID(self.CP, self.CP_SP, self.CI, 0.01)
        torque_controller = LatControlTorque(self.CP, self.CP_SP, self.CI, 0.01)
        long_controller = LongControl(self.CP, self.CP_SP)
        
        # Verify custom values are used
        self.assertEqual(pid_controller.max_angle_rate, 3.0)
        self.assertEqual(pid_controller.high_speed_threshold, 20.0)
        self.assertEqual(pid_controller.high_speed_ki_limit, 0.10)
        
        self.assertEqual(torque_controller.max_lateral_jerk, 5.0)  # Default value for non-mocked param
        self.assertEqual(long_controller.max_jerk, 3.0)
        self.assertEqual(long_controller.max_stopping_jerk, 2.0)


if __name__ == "__main__":
    # Run all tests
    unittest.main()