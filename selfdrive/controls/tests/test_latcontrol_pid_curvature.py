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

    @parameterized.expand([(HONDA.HONDA_CIVIC,), (TOYOTA.TOYOTA_RAV4,)])
    def test_pid_curvature_gain_functionality(self, car_name):
        """Test that the PID controller properly uses curvature gain from CarParamsSP"""
        CarInterface = interfaces[car_name]
        CP = CarInterface.get_non_essential_params(car_name)
        
        # Create custom CarParamsSP with curvature gain interpolation
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]  # Default curvature gain
        
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
        
        # Define a simple curvature gain curve for testing
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.05, 0.1], [1.0, 1.5, 2.0]]  # Simple curve
        
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
        
        # Set up curvature gain with reasonable range
        CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
        CP_SP.curvatureGainInterp = [[0.0, 0.02, 0.04], [1.0, 1.2, 1.5]]  # Normal range
        
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


if __name__ == "__main__":
    unittest.main()