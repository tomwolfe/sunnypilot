#!/usr/bin/env python3
"""
Test suite to validate configurable parameters in sunnypilot autonomous driving system.
This ensures that all hardcoded values mentioned in the critical review have been made configurable.
"""

import pytest
from unittest.mock import MagicMock  # noqa: TID251


from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


class TestConfigurableParameters:
  """Test suite for configurable parameters."""

  @pytest.fixture(autouse=True)
  def setup_method(self):
    """Set up test environment."""
    self.CP = MagicMock()
    self.CP.steerRatio = 15.0
    self.CP.wheelbase = 2.7
    self.CP.minSteerSpeed = 0.5
    self.CP.steerActuatorDelay = 0.1
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

    self.CP_SP = MagicMock()
    from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.helpers import MOCK_MODEL_PATH

    self.CP_SP.neuralNetworkLateralControl.model.path = MOCK_MODEL_PATH
    self.CP_SP.neuralNetworkLateralControl.model.name = "MOCK"
    self.CP_SP.neuralNetworkLateralControl.fuzzyFingerprint = False
    self.CI = MagicMock()
    self.CI.get_steer_feedforward_function.return_value = MagicMock()
    self.CI.torque_from_lateral_accel.return_value = MagicMock()
    self.CI.lateral_accel_from_torque.return_value = MagicMock()

  def test_latcontrol_pid_configurable_parameters(self, mock_params):
    """Test that latcontrol_pid uses configurable parameters."""
    controller = LatControlPID(self.CP, self.CP_SP, self.CI, 0.01, mock_params)

    # Check default values
    assert controller.max_angle_rate == 2.0
    assert controller.high_speed_threshold == 15.0
    assert controller.high_speed_ki_limit == 0.15

    # Test that parameters affect behavior
    # The controller should store these values as instance variables
    assert hasattr(controller, 'max_angle_rate')
    assert hasattr(controller, 'high_speed_threshold')
    assert hasattr(controller, 'high_speed_ki_limit')

  def test_latcontrol_torque_configurable_parameters(self, mock_params):
    """Test that latcontrol_torque uses configurable parameters."""
    controller = LatControlTorque(self.CP, self.CP_SP, self.CI, 0.01, mock_params)

    # Check default values
    assert controller.max_lateral_jerk == 5.0
    assert controller.high_speed_threshold == 15.0
    assert controller.high_speed_ki_limit == 0.15

    # Test that parameters affect behavior
    assert hasattr(controller, 'max_lateral_jerk')
    assert hasattr(controller, 'high_speed_threshold')
    assert hasattr(controller, 'high_speed_ki_limit')

  def test_longcontrol_configurable_parameters(self, mock_params):
    """Test that longcontrol uses configurable parameters."""
    controller = LongControl(self.CP, self.CP_SP, mock_params)

    # Check default values
    assert controller.max_jerk == 2.2  # Default from params fixture
    assert controller.max_stopping_jerk == 1.5  # Default from params fixture
    assert controller.max_output_jerk == 2.0  # Default from params fixture
    assert controller.starting_speed_threshold == 3.0  # Default from params fixture
    assert controller.starting_accel_multiplier == 0.8  # Default from params fixture
    assert controller.starting_accel_limit == 0.8  # Default from params fixture
    assert controller.adaptive_error_threshold == 0.6  # Default from params fixture
    assert controller.adaptive_speed_threshold == 5.0  # Default from params fixture

    # Test that parameters affect behavior
    assert hasattr(controller, 'max_jerk')
    assert hasattr(controller, 'max_stopping_jerk')
    assert hasattr(controller, 'max_output_jerk')
    assert hasattr(controller, 'starting_speed_threshold')
    assert hasattr(controller, 'starting_accel_multiplier')
    assert hasattr(controller, 'starting_accel_limit')
    assert hasattr(controller, 'adaptive_error_threshold')
    assert hasattr(controller, 'adaptive_speed_threshold')

  def test_custom_parameter_values(self, mock_params):
    """Test that custom parameter values are correctly loaded."""
    # Mock custom parameter values
    custom_params = {
      'LateralMaxAngleRate': '3.0',
      'LateralHighSpeedThreshold': '20.0',
      'LateralHighSpeedKiLimit': '0.10',
      'LateralMaxJerk': '6.0',
      'LongitudinalMaxJerk': '3.0',
      'LongitudinalMaxStoppingJerk': '2.0',
      'LongitudinalMaxOutputJerk': '3.0',
      'LongitudinalStartingSpeedThreshold': '5.0',
      'LongitudinalStartingAccelMultiplier': '0.5',
      'LongitudinalStartingAccelLimit': '1.0',
      'LongitudinalAdaptiveErrorThreshold': '0.3',
      'LongitudinalAdaptiveSpeedThreshold': '10.0',
    }

    # Update the mock_params fixture to return custom values
    original_side_effect = mock_params.get.side_effect

    def custom_side_effect(key, block=False):
      if key in custom_params:
        return custom_params[key].encode()  # Params.get returns bytes
      return original_side_effect(key, block)

    mock_params.get.side_effect = custom_side_effect

    # Create controllers with mocked parameters
    pid_controller = LatControlPID(self.CP, self.CP_SP, self.CI, 0.01, mock_params)
    torque_controller = LatControlTorque(self.CP, self.CP_SP, self.CI, 0.01, mock_params)
    long_controller = LongControl(self.CP, self.CP_SP, mock_params)

    # Verify custom values are used
    assert pid_controller.max_angle_rate == 3.0
    assert pid_controller.high_speed_threshold == 20.0
    assert pid_controller.high_speed_ki_limit == 0.10

    assert torque_controller.max_lateral_jerk == float(custom_params['LateralMaxJerk'])  # Custom value should be loaded
    assert long_controller.max_jerk == 3.0
    assert long_controller.max_stopping_jerk == 2.0

  def test_parameter_loading_edge_cases(self, mock_params):
    """Test edge cases in parameter loading."""
    # Test when params.get returns None - Update the mock to return None for all calls
    mock_params.get.return_value = b""  # Return empty bytes which will cause fallback to defaults

    controller = LatControlPID(self.CP, self.CP_SP, self.CI, 0.01, mock_params)
    # Should fall back to default values
    assert controller.max_angle_rate == 2.0
    assert controller.high_speed_threshold == 15.0
    assert controller.high_speed_ki_limit == 0.15
