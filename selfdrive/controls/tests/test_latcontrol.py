from parameterized import parameterized

from cereal import car, log
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.gm.values import CAR as GM
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.locationd.helpers import Pose
from openpilot.common.mock.generators import generate_livePose
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces


class TestLatControl:

  @parameterized.expand([(HONDA.HONDA_CIVIC, LatControlPID), (TOYOTA.TOYOTA_RAV4, LatControlTorque),
                         (NISSAN.NISSAN_LEAF, LatControlAngle), (GM.CHEVROLET_BOLT_EUV, LatControlTorque)])
  def test_saturation(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    sunnypilot_interfaces.setup_interfaces(CI)
    CP_SP = convert_to_capnp(CP_SP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False

    params = log.LiveParametersData.new_message()

    lp = generate_livePose()
    pose = Pose.from_live_pose(lp.livePose)

    # Saturate for curvature limited and controller limited
    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, pose, True, 0.2)
    assert lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, pose, False, 0.2)
    assert not lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 1, pose, False, 0.2)
    assert lac_log.saturated

  @parameterized.expand([(TOYOTA.TOYOTA_RAV4, LatControlTorque), (GM.CHEVROLET_BOLT_EUV, LatControlTorque)])
  def test_high_curvature_response(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    sunnypilot_interfaces.setup_interfaces(CI)
    CP_SP = convert_to_capnp(CP_SP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 15  # 15 m/s, a moderate speed
    CS.steeringPressed = False

    params = log.LiveParametersData.new_message()

    lp = generate_livePose()
    pose = Pose.from_live_pose(lp.livePose)

    # High curvature scenario
    high_curvature = 0.04
    output_torque_high, _, _ = controller.update(True, CS, VM, params, False, high_curvature, pose, False, 0.2)

    # Low curvature scenario
    low_curvature = 0.005
    output_torque_low, _, _ = controller.update(True, CS, VM, params, False, low_curvature, pose, False, 0.2)

    # Assert that the torque output is higher for high curvature
    assert abs(output_torque_high) > abs(output_torque_low)

  @parameterized.expand([(TOYOTA.TOYOTA_RAV4, LatControlTorque), (GM.CHEVROLET_BOLT_EUV, LatControlTorque)])
  def test_curvature_gain_boundaries(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    sunnypilot_interfaces.setup_interfaces(CI)
    CP_SP = convert_to_capnp(CP_SP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 15  # 15 m/s, a moderate speed
    CS.steeringPressed = False

    params = log.LiveParametersData.new_message()

    lp = generate_livePose()
    pose = Pose.from_live_pose(lp.livePose)

    # To ensure a non-zero error for proportional gain calculation
    # We set a desired lateral accel and a zero measured curvature initially.
    # This ensures lac_log.error is non-zero and lac_log.p is proportional to k_p.
    setpoint_error_value = 1.0 # arbitrary non-zero error value
    controller.lat_accel_request_buffer.append(setpoint_error_value) # Sets expected_lateral_accel
    
    # Dictionary to store proportional gains at different curvatures
    p_gains_at_curvatures = {}

    curvatures_to_test = [0.0, 0.02, 0.04, 0.06, 0.07] # 0.07 to test clamping
    expected_gain_multipliers = [1.0, 1.2, 1.5, 2.0, 2.0] # From CURVATURE_GAIN_INTERP and clamping

    for i, curvature in enumerate(curvatures_to_test):
      controller.pid.reset() # Reset integrator and derivative for clean proportional term check

      # Simulate an update call where desired_curvature drives the PID's curvature,
      # and the error leads to a proportional output.
      # We need to manually set the error for the PID controller to be non-zero
      # and consistent across tests to isolate the curvature gain.
      # The `update` method of LatControlTorque calculates its own error.
      # We need to ensure that `lac_log.error` is constant and non-zero.
      # `setpoint` is effectively `setpoint_error_value` and `measurement` is 0 if `CS.steeringAngleDeg` and `params.roll` are 0.
      # This means `error = setpoint_error_value`.
      CS.steeringAngleDeg = 0.0 # Make measured_curvature zero
      params.roll = 0.0 # Make roll_compensation zero
      
      _, _, lac_log = controller.update(True, CS, VM, params, False, curvature, pose, False, 0.2)
      
      # lac_log.error should be non-zero and consistent due to our setup
      assert lac_log.error != 0, f"Error is zero at curvature {curvature}"
      
      # Calculate the actual effective k_p
      actual_k_p_effective = lac_log.p / lac_log.error
      p_gains_at_curvatures[curvature] = actual_k_p_effective

    # Verify the proportional gains based on expected multipliers
    base_k_p = p_gains_at_curvatures[0.0] # This is k_p when curvature gain is 1.0x

    for i, curvature in enumerate(curvatures_to_test):
      expected_k_p = base_k_p * expected_gain_multipliers[i]
      assert abs(p_gains_at_curvatures[curvature] - expected_k_p) < 1e-6, \
        f"Curvature: {curvature}, Expected k_p: {expected_k_p}, Actual k_p: {p_gains_at_curvatures[curvature]}"

    # Also assert the monotonicity of gains for sanity check
    assert p_gains_at_curvatures[0.0] < p_gains_at_curvatures[0.02]
    assert p_gains_at_curvatures[0.02] < p_gains_at_curvatures[0.04]
    assert p_gains_at_curvatures[0.04] < p_gains_at_curvatures[0.06]
    assert p_gains_at_curvatures[0.06] == p_gains_at_curvatures[0.07] # Clamping
