import unittest
from unittest.mock import Mock, patch
import numpy as np

from cereal import car, log
import cereal.messaging as messaging

# Assuming controlsd.py is in the same directory as this test file for import
# Adjust import path if needed based on the actual project structure.
# For now, let's assume `controlsd.py` is in `selfdrive/controls/`
# and this test file is in `selfdrive/controls/tests/`
from openpilot.selfdrive.controls.controlsd import Controls, State, LaneChangeState, LaneChangeDirection
from openpilot.common.swaglog import cloudlog


class TestControlsd(unittest.TestCase):

    def setUp(self):
        # Mock necessary dependencies for Controls instantiation
        self.mock_params = Mock()
        self.mock_car_params = Mock()
        self.mock_car_params.carFingerprint = "TOYOTA_RAV4" # Example
        self.mock_car_params.lateralTuning.which.return_value = 'pid' # Example
        self.mock_car_params.steerControlType = car.CarParams.SteerControlType.torque # Example
        self.mock_car_params.minSteerSpeed = 0.5
        self.mock_car_params.openpilotLongitudinalControl = True
        self.mock_car_params.pcmCruiseSpeed = False

        # Mock CI (CarInterface)
        self.mock_ci = Mock()
        self.mock_ci.get_pid_accel_limits.return_value = (-5.0, 2.0) # Default accel limits

        # Patch interfaces to return our mock CI
        self.patch_interfaces = patch('opendbc.car.car_helpers.interfaces', {self.mock_car_params.carFingerprint: lambda CP, CP_SP: self.mock_ci})
        self.patch_interfaces.start()

        # Mock messaging.log_from_bytes for CarParams
        self.patch_log_from_bytes = patch('cereal.messaging.log_from_bytes', return_value=self.mock_car_params)
        self.patch_log_from_bytes.start()
        
        # Patch Params.get method to control returned values
        self.patch_params_get = patch.object(self.mock_params, 'get')
        self.mock_params_get = self.patch_params_get.start()
        self.mock_params_get.side_effect = self._mock_params_get_side_effect

        with patch('openpilot.common.params.Params', return_value=self.mock_params):
            # Instantiate Controls
            self.controls = Controls()

        # Mock SubMaster and PubMaster
        self.controls.sm = Mock()
        self.controls.pm = Mock()
        self.controls.LoC = Mock() # Mock LongControl

        # Set initial conditions for sm
        self.controls.sm.updated = {'liveCalibration': False, 'livePose': False, 'modelV2': False, 'radarState': False, 'carState': False, 'carControl': False, 'longitudinalPlan': False, 'selfdriveState': False, 'liveParameters': False, 'liveTorqueParameters': False}
        self.controls.sm.all_checks.return_value = True # Assume all checks pass for safety monitor
        self.controls.sm.logMonoTime = {'longitudinalPlan': 0, 'modelV2': 0}

        # Mock carState
        self.mock_car_state = Mock()
        self.mock_car_state.vEgo = 10.0 # m/s
        self.mock_car_state.standstill = False
        self.mock_car_state.steeringAngleDeg = 0.0
        self.mock_car_state.vCruise = 10.0
        self.mock_car_state.steerFaultTemporary = False
        self.mock_car_state.steerFaultPermanent = False
        self.mock_car_state.canValid = True

        # Mock modelV2
        self.mock_model_v2 = Mock()
        self.mock_model_v2.meta.laneChangeState = LaneChangeState.off
        self.mock_model_v2.action.desiredCurvature = 0.0
        self.mock_model_v2.meta.laneChangeDirection = LaneChangeDirection.none

        # Mock longitudinalPlan
        self.mock_long_plan = Mock()
        self.mock_long_plan.aTarget = 0.0
        self.mock_long_plan.shouldStop = False
        self.mock_long_plan.hasLead = False

        # Mock liveParameters
        self.mock_live_parameters = Mock()
        self.mock_live_parameters.stiffnessFactor = 1.0
        self.mock_live_parameters.steerRatio = 15.0
        self.mock_live_parameters.angleOffsetDeg = 0.0
        self.mock_live_parameters.roll = 0.0

        # Mock selfdriveState
        self.mock_selfdrive_state = Mock(enabled=True)
        self.mock_selfdrive_state.personality.raw = 0 # Example value
        self.mock_selfdrive_state.alertHudVisual = False
        self.mock_selfdrive_state.state = State.disabled # Initial state, will be enabled in tests as needed


        # Set mocks for sm
        self.controls.sm.__getitem__.side_effect = lambda key: {
            'carState': self.mock_car_state,
            'modelV2': self.mock_model_v2,
            'longitudinalPlan': self.mock_long_plan,
            'selfdriveState': self.mock_selfdrive_state,
            'liveParameters': self.mock_live_parameters,
            'radarState': Mock(), # needed for safety_monitor
            'carControl': Mock(), # needed for safety_monitor
            'livePose': Mock(), # needed for safety_monitor
            'onroadEvents': [], # needed for CC.longActive
            'liveCalibration': Mock(),
            'liveTorqueParameters': Mock(useParams=False),
            'driverMonitoringState': Mock(awarenessStatus=1.0),
            'driverAssistance': Mock(leftLaneDeparture=False, rightLaneDeparture=False)
        }.get(key, Mock()) # Return a generic Mock for anything else

        # Set up a mock safety monitor that returns a specific safety score
        self.mock_safety_monitor = Mock()
        self.controls.safety_monitor = self.mock_safety_monitor

    def _mock_params_get_side_effect(self, key, block=False):
        if key == "SafetyCriticalThreshold":
            return "0.2"
        elif key == "SafetyHighRiskThreshold":
            return "0.4"
        elif key == "SafetyModerateRiskThreshold":
            return "0.6"
        elif key == "CarParams":
            return self.mock_car_params.to_bytes() # Needs to return bytes for log_from_bytes
        return None # Default for other params

    def tearDown(self):
        self.patch_interfaces.stop()
        self.patch_log_from_bytes.stop()
        self.patch_params_get.stop()

    def test_braking_safety_factor_preservation(self):
        # Configure safety monitor to trigger degraded mode with a score that activates safety factors
        self.controls.safety_critical_threshold = 0.2
        self.controls.safety_high_risk_threshold = 0.4
        self.controls.safety_moderate_risk_threshold = 0.6

        # Set overall_safety_score to trigger high risk, but not critical (e.g., 0.3)
        # This will set acceleration_safety_factor to 0.6 and braking_safety_factor to 1.0
        self.mock_safety_monitor.update.return_value = (0.3, False, {'overall_safety_score': 0.3})

        # Set original min and max accel limits
        original_min_accel = -5.0
        original_max_accel = 2.0
        self.mock_ci.get_pid_accel_limits.return_value = (original_min_accel, original_max_accel)

        # Ensure safety_degraded_mode is true to enter the safety factor block
        self.controls.safety_degraded_mode = True

        # Call the state_control method
        CC, _ = self.controls.state_control()

        # Extract adjusted accel limits from the returned carControl message
        # These are passed to self.controls.LoC.update(..., pid_accel_limits)
        # So we need to inspect the call to controls.LoC.update
        self.controls.LoC.update.assert_called_once()
        called_pid_accel_limits = self.controls.LoC.update.call_args[0][4]

        adjusted_min_limit = called_pid_accel_limits[0]
        adjusted_max_limit = called_pid_accel_limits[1]

        # Assertions
        # Braking capability must be fully preserved
        self.assertAlmostEqual(adjusted_min_limit, original_min_accel, places=5, msg="Braking capability should not be reduced.")

        # Acceleration capability should be reduced by acceleration_safety_factor (0.6 for this scenario)
        expected_max_accel = original_max_accel * 0.6
        self.assertAlmostEqual(adjusted_max_limit, expected_max_accel, places=5, msg="Acceleration capability should be reduced.")

    def test_min_accel_validation_warning(self):
        # Temporarily suppress cloudlog warnings for this test
        with patch.object(cloudlog, 'warning') as mock_warning:
            self.controls.safety_critical_threshold = 0.2
            self.controls.safety_high_risk_threshold = 0.4
            self.controls.safety_moderate_risk_threshold = 0.6
            self.mock_safety_monitor.update.return_value = (0.3, False, {'overall_safety_score': 0.3})
            self.controls.safety_degraded_mode = True

            # Set original min_accel to a value that would trigger the warning (e.g., -3.0 which is > -4.0)
            original_min_accel = -3.0
            original_max_accel = 2.0
            self.mock_ci.get_pid_accel_limits.return_value = (original_min_accel, original_max_accel)

            CC, _ = self.controls.state_control()
            mock_warning.assert_called_with(
                f"Safety Validation: adjusted_min_limit {original_min_accel:.2f} m/s^2 "
                f"is weaker than safe threshold {-4.0:.2f} m/s^2."
            )

    def test_max_accel_validation_warning(self):
        with patch.object(cloudlog, 'warning') as mock_warning:
            self.controls.safety_critical_threshold = 0.2
            self.controls.safety_high_risk_threshold = 0.4
            self.controls.safety_moderate_risk_threshold = 0.6
            self.mock_safety_monitor.update.return_value = (0.3, False, {'overall_safety_score': 0.3})
            self.controls.safety_degraded_mode = True

            original_min_accel = -5.0
            # Set original max_accel to a value that would trigger the warning (e.g., 3.0 which is > 2.0)
            original_max_accel = 3.0
            self.mock_ci.get_pid_accel_limits.return_value = (original_min_accel, original_max_accel)

            CC, _ = self.controls.state_control()
            # The acceleration_safety_factor for overall_safety_score 0.3 is 0.6
            expected_adjusted_max_accel = original_max_accel * 0.6
            mock_warning.assert_called_with(
                f"Safety Validation: adjusted_max_limit {expected_adjusted_max_accel:.2f} m/s^2 "
                f"is stronger than safe threshold {2.0:.2f} m/s^2."
            )

if __name__ == '__main__':
    unittest.main()
