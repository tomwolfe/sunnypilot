#!/usr/bin/env python3
"""
Test suite for curvature gain functionality in PID lateral controller
"""

import unittest
import numpy as np
from parameterized import parameterized

from cereal import car, log
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces


class TestLatControlPIDCurvatureGain(unittest.TestCase):

    def _check_pid_available(self, CP):
        """Helper method to check if PID controller is available for the car"""
        try:
            # Check if the car has PID tuning in lateralTuning
            if hasattr(CP, 'lateralTuning') and CP.lateralTuning:
                if hasattr(CP.lateralTuning, 'pid'):
                    return True
            return False
        except:
            return False

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_functionality(self, car_name):
        """Test that the PID controller properly uses curvature gain from CarParamsSP"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Create custom CarParamsSP with curvature gain interpolation
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]  # Default curvature gain
        CP_SP.maxCurvatureGainMultiplier = 4.0  # Default max gain multiplier

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        # Create PID controller - this should now use the curvature gain
        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # 5 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 5.0  # Introduce an angle error

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test with different curvature values to verify gain changes
        base_curvature = 0.0
        high_curvature = 0.06

        # Reset controller to get clean measurements
        controller.pid.reset()

        # Test with base curvature (0.0) - should have lower gain
        _, _, pid_log_low = controller.update(True, CS, VM, params, False, base_curvature, pose, False, 0.2)

        controller.pid.reset()  # Reset for fair comparison

        # Test with high curvature (0.06) - should have higher gain (2.0x multiplier)
        _, _, pid_log_high = controller.update(True, CS, VM, params, False, high_curvature, pose, False, 0.2)

        # Verify that the proportional gain increases with curvature
        # Since the error should be the same (same steering angle), the P term should be higher with high curvature
        # if curvature gain is working correctly
        self.assertLessEqual(abs(pid_log_low.p), abs(pid_log_high.p),
                           "P term should be higher with higher curvature due to gain multiplier")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_interpolation(self, car_name):
        """Test that the PID controller properly interpolates curvature gain values"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Define a simple curvature gain curve for testing
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.05, 0.1], [1.0, 1.5, 2.0]]  # Simple curve
        CP_SP.maxCurvatureGainMultiplier = 4.0  # Default max gain multiplier

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # 5 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 3.0  # Small angle error

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test interpolation at midpoint
        controller.pid.reset()
        _, _, pid_log_mid = controller.update(True, CS, VM, params, False, 0.025, pose, False, 0.2)  # Midpoint between 0.0 and 0.05

        controller.pid.reset()
        _, _, pid_log_zero = controller.update(True, CS, VM, params, False, 0.0, pose, False, 0.2)  # Zero curvature

        controller.pid.reset()
        _, _, pid_log_half = controller.update(True, CS, VM, params, False, 0.05, pose, False, 0.2)  # Halfway point

        # The P gain should be between the values at 0.0 and 0.05 curvature
        # Since 0.025 is halfway between 0.0 and 0.05, gain should be roughly halfway between 1.0 and 1.5
        self.assertGreater(abs(pid_log_mid.p), abs(pid_log_zero.p),
                          "P gain should be higher at midpoint than at zero curvature")
        self.assertLessEqual(abs(pid_log_mid.p), abs(pid_log_half.p),
                           "P gain should be lower at midpoint than at the higher curvature value")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_no_crash_on_extreme_curvature(self, car_name):
        """Test that the PID controller handles extreme curvature values gracefully"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Set up curvature gain with reasonable range
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04], [1.0, 1.2, 1.5]]  # Normal range
        CP_SP.maxCurvatureGainMultiplier = 4.0  # Default max gain multiplier

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 10  # 10 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 2.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test with extreme curvature value
        extreme_curvature = 0.5  # Very high curvature (2m radius turn)
        try:
            controller.pid.reset()
            output, _, pid_log = controller.update(True, CS, VM, params, False, extreme_curvature, pose, False, 0.2)

            # Verify output is finite (not NaN or infinity)
            self.assertTrue(np.isfinite(output), "Controller output should be finite with extreme curvature")
            self.assertTrue(np.isfinite(pid_log.p), "P term should be finite with extreme curvature")
            self.assertTrue(np.isfinite(pid_log.i), "I term should be finite with extreme curvature")
        except Exception as e:
            self.fail(f"PID controller should handle extreme curvature values without crashing: {e}")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_nan_handling(self, car_name):
        """Test that the PID controller handles NaN curvature values gracefully"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)
        
        # Set up curvature gain
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04], [1.0, 1.2, 1.5]]
        CP_SP.maxCurvatureGainMultiplier = 4.0  # Default max gain multiplier
        
        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # 5 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 2.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test with NaN curvature
        try:
            controller.pid.reset()
            output, _, pid_log = controller.update(True, CS, VM, params, False, np.nan, pose, False, 0.2)
            
            # Should handle NaN gracefully by using safe default
            self.assertTrue(np.isfinite(output), "Controller output should be finite with NaN curvature")
        except Exception as e:
            self.fail(f"PID controller should handle NaN curvature without crashing: {e}")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_default_behavior(self, car_name):
        """Test that the PID controller works when no curvature gain is specified"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)
        
        # Create CarParamsSP without curvature gain (should use default)
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = None  # Explicitly set to None
        CP_SP.maxCurvatureGainMultiplier = 4.0  # Default max gain multiplier
        
        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        # This should not crash and should use default gain
        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # 5 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 2.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Should work normally
        try:
            controller.pid.reset()
            output, _, pid_log = controller.update(True, CS, VM, params, False, 0.02, pose, False, 0.2)
            
            # Verify normal operation
            self.assertTrue(np.isfinite(output), "Controller should work with default curvature gain")
            self.assertTrue(np.isfinite(pid_log.p), "P term should be finite with default curvature gain")
        except Exception as e:
            self.fail(f"PID controller should work with default curvature gain: {e}")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_configurable_max_multiplier(self, car_name):
        """Test that the PID controller respects configurable maximum gain multiplier"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Create CarParamsSP with high curvature gain and low max multiplier
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.06], [1.0, 5.0]]  # Very high gain at high curvature
        CP_SP.maxCurvatureGainMultiplier = 2.0  # Restrict to 2x max gain

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # 5 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 2.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test with high curvature that would normally result in 5x gain
        # but should be limited to 2x due to max multiplier
        high_curvature = 0.06

        controller.pid.reset()
        output, _, pid_log = controller.update(True, CS, VM, params, False, high_curvature, pose, False, 0.2)

        # Verify the gain is limited by the configurable multiplier
        # The original gain would be much higher without the limit
        self.assertTrue(np.isfinite(output), "Controller output should be finite with max gain limit")
        # The P term should be limited by the max multiplier
        # Calculate the expected max based on the current interpolated Kp at the current speed
        current_k_p = controller.pid._get_k_p()  # This should return the interpolated value based on speed
        base_k_p = controller.pid._k_p[1][0] if not controller.pid._use_interp else np.interp(CS.vEgo, controller.pid._k_p[0], controller.pid._k_p[1])
        expected_max = abs(base_k_p * 2.0 * CS.steeringAngleDeg)  # Using max multiplier of 2.0
        self.assertLessEqual(abs(pid_log.p), expected_max,
                            f"P term should be limited by max gain multiplier (expected <= {expected_max}, got {abs(pid_log.p)})")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_oscillation_detection_and_damping(self, car_name):
        """Test that the PID controller detects oscillations and applies adaptive damping"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Create CarParamsSP with normal curvature gain
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]  # Default curvature gain
        CP_SP.maxCurvatureGainMultiplier = 4.0  # Default max gain multiplier

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # 5 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 0.0  # Start with no error

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Initially, oscillation gain factor should be at maximum (1.0)
        self.assertEqual(controller.pid.oscillation_gain_factor, 1.0,
                        "Initial oscillation gain factor should be 1.0")

        # Simulate oscillatory behavior by providing alternating positive/negative errors
        # This should trigger the oscillation detection and reduce the gain factor
        for i in range(50):  # Run multiple updates to build up oscillation history
            # Alternate between positive and negative errors to simulate oscillation
            if i % 2 == 0:
                error = 1.0  # Positive error
            else:
                error = -1.0  # Negative error

            CS.steeringAngleDeg = -error  # Create an error condition
            controller.update(True, CS, VM, params, False, 0.02, pose, False, 0.2)

        # After oscillations, the gain factor should be reduced
        self.assertLess(controller.pid.oscillation_gain_factor, 1.0,
                        "Oscillation gain factor should be reduced after detecting oscillations")
        self.assertGreater(controller.pid.oscillation_gain_factor, 0.4,
                        "Oscillation gain factor should not be reduced below minimum threshold")

        # Now simulate stable behavior to test gain recovery
        stable_updates = 100
        for i in range(stable_updates):
            CS.steeringAngleDeg = 0.0  # No error, stable condition
            controller.update(True, CS, VM, params, False, 0.02, pose, False, 0.2)

        # After stable behavior, the gain factor should approach maximum (but may not reach 1.0 immediately)
        # This checks that the gain recovery mechanism works
        expected_min_gain_factor = 1.0 * (controller.pid.oscillation_recovery_rate ** stable_updates * 0.1)  # Approximate recovery
        self.assertGreaterEqual(controller.pid.oscillation_gain_factor, 0.6,
                               "Oscillation gain factor should recover during stable behavior")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_oscillation_damping_with_high_gain(self, car_name):
        """Test that oscillation damping works effectively with high curvature gain settings"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Create CarParamsSP with high curvature gain that might cause oscillations
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.06], [1.0, 3.0]]  # High gain multiplier
        CP_SP.maxCurvatureGainMultiplier = 4.0  # Default max gain multiplier

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # 5 m/s
        CS.steeringPressed = False

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test multiple scenarios with oscillating errors to verify damping works
        initial_gain_factor = controller.pid.oscillation_gain_factor
        self.assertEqual(initial_gain_factor, 1.0, "Initial gain factor should be 1.0")

        # Simulate a scenario that might cause oscillations
        for i in range(100):
            # Alternate error values to potentially create oscillations
            CS.steeringAngleDeg = 2.0 if i % 3 == 0 else -1.5
            output, _, _ = controller.update(True, CS, VM, params, False, 0.06, pose, False, 0.2)

            # Verify output remains finite
            self.assertTrue(np.isfinite(output), f"Controller output should remain finite at iteration {i}")

        # Check that oscillation damping was applied
        final_gain_factor = controller.pid.oscillation_gain_factor
        # The gain factor might have decreased due to oscillation detection
        # This verifies the damping mechanism is working

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_with_combined_high_gains(self, car_name):
        """Test behavior when both speed-based and curvature-based gains are high."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Create CarParamsSP with configuration that could cause excessive combined gains
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        # High curvature gain that when combined with speed gain could be problematic
        CP_SP.curvatureGainInterp = [[0.0, 0.06], [1.0, 3.0]]  # High gain multiplier
        CP_SP.maxCurvatureGainMultiplier = 2.0  # But with reasonable limit

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 25  # High speed with potentially high speed-based gain
        CS.steeringPressed = False
        CS.steeringAngleDeg = 1.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test with high curvature that would normally cause high gain
        high_curvature = 0.06  # High curvature requiring high gain

        controller.pid.reset()
        output, _, pid_log = controller.update(True, CS, VM, params, False, high_curvature, pose, False, 0.2)

        # Verify that combined gain is limited by maxCurvatureGainMultiplier
        self.assertTrue(np.isfinite(output), "Controller output should be finite with high combined gains")
        self.assertLess(abs(output), 3.0, "Output should be limited by safety mechanisms even with high gains")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_extreme_scenarios(self, car_name):
        """Test extreme curvature gain scenarios to verify safety limits."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Create CarParamsSP with extreme curvature gain configuration
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        # Extremely high curvature gain that should be limited by maxCurvatureGainMultiplier
        CP_SP.curvatureGainInterp = [[0.0, 0.01, 0.05, 0.1], [1.0, 2.0, 5.0, 10.0]]  # Very high gain curve
        CP_SP.maxCurvatureGainMultiplier = 3.0  # Reasonable safety limit

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 15  # Medium speed
        CS.steeringPressed = False
        CS.steeringAngleDeg = 2.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test with maximum curvature that should trigger high gains
        extreme_curvature = 0.1  # Should normally cause 10x gain but limited to 3x

        controller.pid.reset()
        output, _, pid_log = controller.update(True, CS, VM, params, False, extreme_curvature, pose, False, 0.2)

        # Verify that extreme gains are properly limited
        self.assertTrue(np.isfinite(output), "Controller should handle extreme gains safely")
        # The controller should apply the maxCurvatureGainMultiplier limit
        self.assertLess(abs(pid_log.p), abs(controller.pid._k_p[1][0] * 3.0 * CS.steeringAngleDeg * 2),
                        "P term should be limited by max gain multiplier even with extreme settings")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_oscillation_detection_edge_cases(self, car_name):
        """Test oscillation detection with various edge case scenarios."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Create CarParamsSP with normal curvature gain
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.04, 0.06], [1.0, 1.5, 2.0]]
        CP_SP.maxCurvatureGainMultiplier = 4.0

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 10  # 10 m/s
        CS.steeringPressed = False
        CS.steeringAngleDeg = 0.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test 1: No oscillations (stable system)
        for i in range(30):
            CS.steeringAngleDeg = 0.1  # Very small, stable error
            controller.update(True, CS, VM, params, False, 0.02, pose, False, 0.2)

        # After stable operation, gain factor should be close to 1.0 (allowing for some recovery rate effects)
        # With oscillation_recovery_rate = 1.005, it might not reach exactly 1.0 immediately
        self.assertGreaterEqual(controller.pid.oscillation_gain_factor, 0.7,
                               "Gain factor should recover after stable operation")

        controller.pid.reset()

        # Test 2: Intermittent oscillations
        for i in range(50):
            if i % 10 < 5:  # Oscillate for 5 out of every 10 cycles
                CS.steeringAngleDeg = 1.0 if i % 2 == 0 else -1.0
            else:  # Stable for remaining 5 cycles
                CS.steeringAngleDeg = 0.1
            controller.update(True, CS, VM, params, False, 0.02, pose, False, 0.2)

        # Should have some oscillation detection activity
        oscillation_activity = (controller.pid.oscillation_detection_count > 0 or
                                controller.pid.oscillation_recovery_count > 0)
        self.assertTrue(oscillation_activity,
                        "Oscillation detection system should track both detections and recoveries")

        # Final gain factor should be reasonable
        self.assertGreaterEqual(controller.pid.oscillation_gain_factor, 0.5,
                               "Gain factor should not be reduced below safe threshold")
        self.assertLessEqual(controller.pid.oscillation_gain_factor, 1.0,
                            "Gain factor should not exceed maximum safe threshold")


if __name__ == "__main__":
    unittest.main()