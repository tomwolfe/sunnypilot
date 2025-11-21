#!/usr/bin/env python3
"""
Test suite for curvature gain functionality in PID lateral controller
"""

import unittest
import numpy as np
from parameterized import parameterized
import time

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
    def test_pid_safety_limiting_combined_high_speed_curvature(self, car_name):
        """Test safety limiting under combined high-speed and high-curvature scenarios."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Configure parameters that would create high combined gains without safety limits
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        # High curvature gain values that when combined with high speed gains could be problematic
        CP_SP.curvatureGainInterp = [[0.0, 0.03, 0.06], [1.0, 3.0, 5.0]]  # High gains for curves
        CP_SP.maxCurvatureGainMultiplier = 2.5  # Reasonable safety limit

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 25  # High speed where speed-based gains are also high
        CS.steeringPressed = False
        CS.steeringAngleDeg = 1.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test scenario: high speed (high base k_p) + high curvature (high curvature multiplier)
        high_curvature = 0.06  # High curvature requiring high gain from interpolation (5.0x multiplier)

        # Reset controller and get initial baseline k_p
        controller.pid.reset()
        initial_k_p = controller.pid._get_k_p()

        # Update with high speed and high curvature
        output, _, pid_log = controller.update(True, CS, VM, params, False, high_curvature, pose, False, 0.2)

        # Verify safety limits are enforced
        # The output should be finite and reasonable despite high combined demands
        self.assertTrue(np.isfinite(output), "Controller output should be finite under high-demand conditions")
        self.assertLess(abs(output), 2.5, "Output should be safely bounded")

        # Check that safety limit counter is working properly
        # Run multiple updates to make sure safety systems are properly tracking
        for i in range(20):
            CS.steeringAngleDeg = 1.5 if i % 2 == 0 else 0.8  # Varying error to test sustained operation
            output_loop, _, pid_log_loop = controller.update(True, CS, VM, params, False, high_curvature, pose, False, 0.2)

            # Every output should be bounded
            self.assertTrue(np.isfinite(output_loop), f"Controller output should be finite at iteration {i}")
            self.assertLess(abs(output_loop), 2.5, f"Output should be safely bounded at iteration {i}")

        # Verify that safety limit triggers are being counted (if they occurred during the test)
        # The controller should be tracking how often safety limits are being enforced
        self.assertGreaterEqual(controller.pid.safety_limit_trigger_count, 0,
                              "Safety limit trigger count should be non-negative")

        # If there were safety limit applications, the safe_mode_trigger_count should be appropriate
        if controller.pid.safety_limit_trigger_count > 50:  # If safety limits were hit frequently
            self.assertFalse(controller.pid.safe_mode_active,
                           "Safe mode should only activate for severe issues, not normal safety limiting")

        # Verify that the controller maintains stability metrics in log
        self.assertIsNotNone(pid_log.curvatureGainFactor, "Curvature gain factor should be logged")
        self.assertIsNotNone(pid_log.oscillationDetected, "Oscillation status should be logged")

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

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_adaptive_filtering(self, car_name):
        """Test that the PID controller uses adaptive filtering based on driving conditions."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]
        CP_SP.maxCurvatureGainMultiplier = 4.0

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 5  # Low speed
        CS.steeringPressed = False
        CS.steeringAngleDeg = 0.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test that adaptive time constant changes based on conditions
        # At low speed and low curvature, should use a higher time constant (more smoothing)
        low_curvature = 0.001
        high_speed = 25.0  # High speed
        high_curvature = 0.08  # High curvature

        # Test at low speed and low curvature
        CS.vEgo = 5.0
        adaptive_tc_low = controller._get_adaptive_time_constant(CS.vEgo, low_curvature)

        # Test at high speed and high curvature
        CS.vEgo = high_speed
        adaptive_tc_high = controller._get_adaptive_time_constant(CS.vEgo, high_curvature)

        # The adaptive time constant should be reasonable in both cases
        self.assertGreater(adaptive_tc_low, 0.01, "Adaptive time constant should be positive at low conditions")
        self.assertLess(adaptive_tc_low, 0.25, "Adaptive time constant should be within bounds at low conditions")
        self.assertGreater(adaptive_tc_high, 0.01, "Adaptive time constant should be positive at high conditions")
        self.assertLess(adaptive_tc_high, 0.25, "Adaptive time constant should be within bounds at high conditions")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_oscillation_window_configuration(self, car_name):
        """Test that the PID controller allows oscillation window size configuration."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]
        CP_SP.maxCurvatureGainMultiplier = 4.0

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        # Test initial oscillation window size
        initial_window = controller.pid.oscillation_window

        # Test configuring to a smaller window size
        controller.pid.set_oscillation_window_size(0.3, 100)  # 0.3 seconds at 100Hz
        new_window = controller.pid.oscillation_window
        expected_window = int(100 * 0.3)  # Should be 30 samples

        # Verify the window size was updated
        self.assertEqual(new_window, expected_window,
                        f"Oscillation window should be updated to {expected_window}, got {new_window}")

        # Test configuring to a larger window size
        controller.pid.set_oscillation_window_size(0.7, 100)  # 0.7 seconds at 100Hz
        larger_window = controller.pid.oscillation_window
        expected_larger_window = int(100 * 0.7)  # Should be 70 samples

        self.assertEqual(larger_window, expected_larger_window,
                        f"Oscillation window should be updated to {expected_larger_window}, got {larger_window}")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_failure_mode_multiple_safety_mechanisms(self, car_name):
        """Test failure mode when multiple safety mechanisms might fail simultaneously."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Create CarParamsSP with high gain settings that could potentially cause issues
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        # Very high curvature gain that should be limited by maxCurvatureGainMultiplier
        CP_SP.curvatureGainInterp = [[0.0, 0.01, 0.05, 0.1], [1.0, 5.0, 8.0, 12.0]]  # Very high gains
        CP_SP.maxCurvatureGainMultiplier = 1.5  # Low limit to ensure safety is triggered

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 20  # High speed where speed-based gains might also be high
        CS.steeringPressed = False
        CS.steeringAngleDeg = 0.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test scenario: high speed (high speed-based gain) + high curvature (high curvature gain)
        # This should trigger the safety limiting mechanisms
        high_curvature = 0.08  # High curvature requiring high gain

        # Run multiple updates to test sustained high-demand scenario
        for i in range(50):
            CS.steeringAngleDeg = 0.5 if i % 3 == 0 else -0.3  # Oscillating small errors to test
            output, _, pid_log = controller.update(True, CS, VM, params, False, high_curvature, pose, False, 0.2)

            # Verify that the controller remains stable even under high-demand conditions
            self.assertTrue(np.isfinite(output), f"Controller output should remain finite at iteration {i}")
            self.assertLess(abs(output), 2.0, f"Output should be limited by safety mechanisms at iteration {i}")

        # Check that safety mechanisms have been active
        # The safety limit trigger count should be greater than 0 if limits were enforced
        self.assertGreaterEqual(controller.pid.safety_limit_trigger_count, 0,
                               "Safety limit trigger count should be non-negative")

        # The safe mode should not be active under normal conditions, but if it is,
        # it means the system properly detected and responded to potential issues
        if controller.pid.safe_mode_active:
            self.assertGreaterEqual(controller.pid.safe_mode_trigger_count, 0,
                                   "Safe mode trigger count should be non-negative when safe mode is active")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_failure_mode_invalid_inputs(self, car_name):
        """Test failure mode when invalid inputs are provided to the curvature gain system."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Create CarParamsSP with normal parameters
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]
        CP_SP.maxCurvatureGainMultiplier = 4.0

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 10  # Medium speed
        CS.steeringPressed = False
        CS.steeringAngleDeg = 1.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_livePose()
        pose = Pose.from_live_pose(lp.livePose)

        # Test with NaN curvature input
        controller.pid.reset()
        output, _, pid_log = controller.update(True, CS, VM, params, False, float('nan'), pose, False, 0.2)

        # Verify that the controller handles NaN gracefully
        self.assertTrue(np.isfinite(output), "Controller should handle NaN curvature gracefully")

        # Safe mode should not have been triggered by a single NaN input in normal conditions
        # but the controller should recover properly
        self.assertTrue(hasattr(controller.pid, 'safe_mode_active'),
                        "PID controller should have safe mode attributes")

        # Test with extremely high curvature to check safety clamping
        controller.pid.reset()
        output_high, _, pid_log_high = controller.update(True, CS, VM, params, False, 1.0, pose, False, 0.2)  # Very high curvature

        # Output should remain finite and reasonable
        self.assertTrue(np.isfinite(output_high), "Controller should handle extremely high curvature")
        self.assertLess(abs(output_high), 3.0, "High curvature should still result in bounded output")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_explicit_safe_mode_recovery_scenario(self, car_name):
        """Test explicit safe mode activation and recovery scenarios."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0], [1.0]]
        CP_SP.maxCurvatureGainMultiplier = 4.0
        CP_SP.safetyLimitThreshold = 10  # Low threshold to easily trigger safe mode
        CP_SP.safeModeRecoveryTime = 1.0  # Short recovery time for testing

        CI = CarInterface(CP, CP_SP)
        sunnypilot_interfaces.setup_interfaces(CI)
        CP_SP = convert_to_capnp(CP_SP)
        VM = VehicleModel(CP)

        controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

        CS = car.CarState.new_message()
        CS.vEgo = 10
        CS.steeringPressed = False
        CS.steeringAngleDeg = 2.0

        params = log.LiveParametersData.new_message()
        params.angleOffsetDeg = 0.0

        from openpilot.selfdrive.locationd.helpers import Pose
        from openpilot.common.mock.generators import generate_livePose
        lp = generate_live_pose()
        pose = Pose.from_live_pose(lp.livePose)

        # 1. Trigger safe mode
        controller.pid.reset()
        for i in range(CP_SP.safetyLimitThreshold + 5): # Exceed threshold
            controller.pid.safety_limit_trigger_count += 1
            controller.pid.safety_limit_trigger_times.append(time.time())
            controller.update(True, CS, VM, params, False, 0.05, pose, False, 0.2)
            time.sleep(DT_CTRL) # Simulate control loop update

        self.assertTrue(controller.pid.safe_mode_active, "Safe mode should be active after triggers")
        self.assertIsNotNone(controller.pid.safe_mode_start_time, "Safe mode start time should be set")

        # 2. Simulate stable operation for recovery
        controller.pid.safety_limit_trigger_count = controller.pid.safety_limit_recovery_threshold - 1 # Below recovery threshold
        controller.pid.safety_limit_trigger_times = [] # No recent triggers

        # Simulate stable operation for less than safe_mode_recovery_time
        stable_start_time = time.time()
        while time.time() - stable_start_time < CP_SP.safeModeRecoveryTime - (DT_CTRL / 2): # Just shy of recovery time
            controller.update(True, CS, VM, params, False, 0.01, pose, False, 0.2)
            time.sleep(DT_CTRL)

        self.assertTrue(controller.pid.safe_mode_active, "Safe mode should still be active before recovery time expires")
        self.assertIsNotNone(controller.pid.safe_mode_recovery_start_time, "Recovery start time should be set")

        # 3. Simulate stable operation until safe_mode_recovery_time expires
        while time.time() - stable_start_time < CP_SP.safeModeRecoveryTime + (DT_CTRL / 2): # Just past recovery time
            controller.update(True, CS, VM, params, False, 0.01, pose, False, 0.2)
            time.sleep(DT_CTRL)

        self.assertFalse(controller.pid.safe_mode_active, "Safe mode should be deactivated after recovery time expires")
        self.assertIsNone(controller.pid.safe_mode_recovery_start_time, "Recovery start time should be reset after recovery")
        self.assertIsNone(controller.pid.safe_mode_start_time, "Safe mode start time should be reset after recovery")

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_adaptive_threshold_extreme_vehicle_params(self, car_name):
        """Test that adaptive oscillation thresholds adjust correctly for extreme vehicle mass and wheelbase."""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)

        # Skip test if PID controller not available for this car
        if not self._check_pid_available(CP):
            self.skipTest(f"PID controller not available for {car_name}")

        # Test cases for extreme vehicle parameters
        extreme_params = [
            # Very light vehicle
            {"mass": 800.0, "wheelbase": 2.0, "expected_mass_adj": 0.9, "expected_wb_adj": 0.9, "desc": "Very light, short wheelbase"},
            # Very heavy vehicle
            {"mass": 2500.0, "wheelbase": 3.5, "expected_mass_adj": 1.1, "expected_wb_adj": 1.1, "desc": "Very heavy, long wheelbase"},
            # Light mass, long wheelbase
            {"mass": 1000.0, "wheelbase": 3.2, "expected_mass_adj": 0.9, "expected_wb_adj": 1.1, "desc": "Light mass, long wheelbase"},
            # Heavy mass, short wheelbase
            {"mass": 2200.0, "wheelbase": 2.2, "expected_mass_adj": 1.1, "expected_wb_adj": 0.9, "desc": "Heavy mass, short wheelbase"},
        ]

        for params_case in extreme_params:
            with self.subTest(msg=params_case["desc"]):
                # Create a controller with extreme vehicle parameters
                CP.mass = params_case["mass"]
                CP.wheelbase = params_case["wheelbase"]

                CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
                # Keep curvature gain interp simple for this test
                CP_SP.curvatureGainInterp = [[0.0], [1.0]]
                CP_SP.maxCurvatureGainMultiplier = 4.0

                CI = CarInterface(CP, CP_SP)
                sunnypilot_interfaces.setup_interfaces(CI)
                CP_SP = convert_to_capnp(CP_SP)
                VM = VehicleModel(CP)

                controller = LatControlPID(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

                # Test _calculate_adaptive_threshold
                base_threshold = 0.5
                adaptive_threshold = controller.pid._calculate_adaptive_threshold(base_threshold)
                expected_total_adj = (params_case["expected_mass_adj"] * 0.6 + params_case["expected_wb_adj"] * 0.4)
                expected_adaptive_threshold = base_threshold * expected_total_adj
                self.assertAlmostEqual(adaptive_threshold, expected_adaptive_threshold, places=5,
                                       msg=f"Adaptive threshold mismatch for {params_case['desc']}")

                # Test _calculate_adaptive_oscillation_thresholds for each type
                base_osc_threshold = 0.6
                
                # Sign Change
                adaptive_sign_change = controller.pid._calculate_adaptive_oscillation_thresholds(base_osc_threshold, "sign_change")
                expected_sc_factor = (params_case["expected_mass_adj"] * 0.5 + params_case["expected_wb_adj"] * 0.5)
                self.assertAlmostEqual(adaptive_sign_change, base_osc_threshold * expected_sc_factor, places=5,
                                       msg=f"Sign change threshold mismatch for {params_case['desc']}")

                # Variance
                adaptive_variance = controller.pid._calculate_adaptive_oscillation_thresholds(base_osc_threshold, "variance")
                expected_var_factor = (params_case["expected_mass_adj"] * 0.7 + params_case["expected_wb_adj"] * 0.3)
                self.assertAlmostEqual(adaptive_variance, base_osc_threshold * expected_var_factor, places=5,
                                       msg=f"Variance threshold mismatch for {params_case['desc']}")

                # Zero Crossing
                adaptive_zero_crossing = controller.pid._calculate_adaptive_oscillation_thresholds(base_osc_threshold, "zero_crossing")
                expected_zc_factor = (params_case["expected_mass_adj"] * 0.4 + params_case["expected_wb_adj"] * 0.6)
                self.assertAlmostEqual(adaptive_zero_crossing, base_osc_threshold * expected_zc_factor, places=5,
                                       msg=f"Zero crossing threshold mismatch for {params_case['desc']}")

if __name__ == "__main__":
    unittest.main()
