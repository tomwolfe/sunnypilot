import unittest
import numpy as np

from cereal import messaging, custom
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision, VisionState, _ENTERING_PRED_LAT_ACC_TH, _TURNING_LAT_ACC_TH, _LEAVING_LAT_ACC_TH, _FINISH_LAT_ACC_TH, _LEAVING_ACC

class TestVisionController(unittest.TestCase):
  def _create_mock_sm(self, v_ego, orientation_rate_z, curvature):
    sm = messaging.SubMaster(['modelV2', 'controlsState'])

    msg_model = messaging.new_message('modelV2')
    msg_model.modelV2.velocity.x = [v_ego] * len(msg_model.modelV2.velocity.x)
    msg_model.modelV2.orientationRate.z = list(orientation_rate_z)
    sm['modelV2'] = msg_model

    msg_controls_state = messaging.new_message('controlsState')
    msg_controls_state.controlsState.curvature = curvature
    sm['controlsState'] = msg_controls_state
    
    return sm

  def test_smoother_turn_entry_and_exit(self):
    vision_controller = SmartCruiseControlVision()
    v_ego = 20.0
    a_ego = 0.0
    v_cruise_setpoint = 25.0

    # Test Entering state with smoother deceleration
    # Simulate a predicted curve that should trigger the new mid-point in the lookup table
    predicted_lat_accels = np.array([0.0] * 10 + [2.0] * 5 + [0.0] * 15) # Corresponds to max_pred_lat_acc of 2.0
    orientation_rate_z = predicted_lat_accels / v_ego
    sm = self._create_mock_sm(v_ego, orientation_rate_z, 0.0)

    # Update controller to enter the ENTERING state
    vision_controller.update(sm, True, False, v_ego, a_ego, v_cruise_setpoint)
    self.assertEqual(vision_controller.state, VisionState.entering)

    # Check that the target acceleration matches the new interpolated value (-0.6)
    self.assertAlmostEqual(vision_controller.a_target, -0.6, delta=1e-2)

    # Test LEAVING state with smoother acceleration
    # First, force the controller into the TURNING state
    vision_controller.state = VisionState.turning

    # Now, simulate exiting the turn by reducing lateral acceleration
    # We will test a point between _FINISH_LAT_ACC_TH (1.1) and _LEAVING_LAT_ACC_TH (1.3)
    current_lat_acc_sim = 1.2
    curvature_sim = current_lat_acc_sim / (v_ego**2)
    sm = self._create_mock_sm(v_ego, np.zeros(len(sm['modelV2'].orientationRate.z)), curvature_sim)

    # Update will transition from TURNING to LEAVING because current_lat_acc is below _LEAVING_LAT_ACC_TH
    vision_controller.update(sm, True, False, v_ego, a_ego, v_cruise_setpoint)
    self.assertEqual(vision_controller.state, VisionState.leaving)

    # Check that the target acceleration is interpolated correctly
    expected_a_target = np.interp(current_lat_acc_sim, [_FINISH_LAT_ACC_TH, _LEAVING_LAT_ACC_TH], [_LEAVING_ACC, 0.2])
    self.assertAlmostEqual(vision_controller.a_target, expected_a_target, delta=1e-2)

  def test_entering_state_lookup_table(self):
    """Test the new 3-point lookup table in the ENTERING state."""
    vision_controller = SmartCruiseControlVision()
    v_ego = 20.0
    a_ego = 0.0
    v_cruise_setpoint = 25.0

    # Test the original boundary points
    # Test 1.3 m/s^2 -> -0.2 m/s^2 deceleration
    predicted_lat_accels = np.array([0.0] * 10 + [1.3] * 5 + [0.0] * 15)
    orientation_rate_z = predicted_lat_accels / v_ego
    sm = self._create_mock_sm(v_ego, orientation_rate_z, 0.0)

    vision_controller.update(sm, True, False, v_ego, a_ego, v_cruise_setpoint)
    self.assertEqual(vision_controller.state, VisionState.entering)
    self.assertAlmostEqual(vision_controller.a_target, -0.2, delta=1e-2)

    # Test 3.0 m/s^2 -> -1.0 m/s^2 deceleration
    predicted_lat_accels = np.array([0.0] * 10 + [3.0] * 5 + [0.0] * 15)
    orientation_rate_z = predicted_lat_accels / v_ego
    sm = self._create_mock_sm(v_ego, orientation_rate_z, 0.0)

    vision_controller.update(sm, True, False, v_ego, a_ego, v_cruise_setpoint)
    self.assertEqual(vision_controller.state, VisionState.entering)
    self.assertAlmostEqual(vision_controller.a_target, -1.0, delta=1e-2)

    # Test the new middle point: 2.0 m/s^2 -> -0.6 m/s^2 deceleration
    predicted_lat_accels = np.array([0.0] * 10 + [2.0] * 5 + [0.0] * 15)
    orientation_rate_z = predicted_lat_accels / v_ego
    sm = self._create_mock_sm(v_ego, orientation_rate_z, 0.0)

    vision_controller.update(sm, True, False, v_ego, a_ego, v_cruise_setpoint)
    self.assertEqual(vision_controller.state, VisionState.entering)
    self.assertAlmostEqual(vision_controller.a_target, -0.6, delta=1e-2)

  def test_leaving_state_interpolation(self):
    """Test the interpolation logic in the LEAVING state."""
    vision_controller = SmartCruiseControlVision()
    v_ego = 20.0
    a_ego = 0.0
    v_cruise_setpoint = 25.0

    # Test different lateral acceleration values in the LEAVING state
    test_cases = [
      (1.1, 0.5),  # At _FINISH_LAT_ACC_TH, should return _LEAVING_ACC (0.5)
      (1.3, 0.2),  # At _LEAVING_LAT_ACC_TH, should return 0.2
      (1.2, 0.35), # Mid-point between 1.1 and 1.3 should interpolate to ~0.35
    ]

    for lat_acc, expected_acc in test_cases:
      vision_controller.state = VisionState.leaving
      curvature_sim = lat_acc / (v_ego**2)
      sm = self._create_mock_sm(v_ego, np.zeros(len(sm['modelV2'].orientationRate.z)), curvature_sim)

      vision_controller.update(sm, True, False, v_ego, a_ego, v_cruise_setpoint)
      self.assertEqual(vision_controller.state, VisionState.leaving)
      self.assertAlmostEqual(vision_controller.a_target, expected_acc, delta=0.01,
                           msg=f"Failed for lat_acc={lat_acc}, expected ~{expected_acc}, got {vision_controller.a_target}")

if __name__ == "__main__":
  unittest.main()
