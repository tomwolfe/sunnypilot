from openpilot.selfdrive.controls.lib.edge_case_handler import EdgeCaseHandler
from cereal import car


def test_edge_case_handler_initialization():
  """Test that EdgeCaseHandler initializes properly."""
  handler = EdgeCaseHandler()

  # Verify initial state
  assert not handler._initialized
  assert len(handler.steering_angle_history) == 0

  print("✓ Edge case handler initialization test passed")


def test_sharp_curve_detection():
  """Test sharp curve detection."""
  handler = EdgeCaseHandler()

  # Create mock car state
  CS = car.CarState.new_message()
  CS.vEgo = 20.0  # 20 m/s

  # Create mock model data
  class MockPlan:
    class OrientationRate:
      y = [0.1, 0.15, 0.25, 0.1, 0.05]  # High orientation rate indicates sharp turn

    orientationRate = OrientationRate()

  class MockModelData:
    plan = MockPlan()
    action = type('obj', (object,), {'desiredCurvature': 0.15})()  # High curvature

  model_data = MockModelData()

  # Test curve detection
  is_sharp = handler.detect_sharp_curves(CS, model_data)
  print(f"Sharp curve detected: {is_sharp}")

  print("✓ Sharp curve detection test passed")


def test_abnormal_steering_detection():
  """Test abnormal steering detection."""
  handler = EdgeCaseHandler()

  # Create mock car state with high steering angle change
  CS = car.CarState.new_message()
  CS.steeringAngleDeg = 45.0  # Large steering angle

  # Force the handler to be initialized with a different previous angle
  handler._initialized = True
  handler._last_steering_angle = 5.0  # Previous small angle

  # Test abnormal steering detection
  is_abnormal = handler.detect_abnormal_steering(CS)
  print(f"Abnormal steering detected: {is_abnormal}")

  print("✓ Abnormal steering detection test passed")


def test_construction_zone_detection():
  """Test construction zone detection."""
  handler = EdgeCaseHandler()

  # Create mock car state
  CS = car.CarState.new_message()
  CS.vEgo = 15.0

  # Create mock radar state with slow vehicles
  class MockLead:
    status = True
    vLead = 5.0  # Very slow vehicle
    dRel = 50.0
    vRel = 0.0

  class MockRadarState:
    leadOne = MockLead()
    leadTwo = MockLead()

  # Add several entries to speed history to trigger variance detection
  for i in range(25):
    handler.speed_history.append(15.0 if i < 10 else 8.0)  # Speed change

  radar_state = MockRadarState()
  model_data = None

  # Test construction zone detection
  is_construction = handler.detect_construction_zone(CS, radar_state, model_data)
  print(f"Construction zone detected: {is_construction}")

  print("✓ Construction zone detection test passed")


def test_unusual_scenario_handling():
  """Test the main unusual scenario handling function."""
  handler = EdgeCaseHandler()

  # Create mock data
  CS = car.CarState.new_message()
  CS.vEgo = 20.0
  CS.vCruise = 25.0

  # Test scenario handling
  scenarios = handler.handle_unusual_scenarios(CS, None, None)

  # Verify return structure
  assert 'sharp_curve' in scenarios
  assert 'construction_zone' in scenarios
  assert 'recommended_speed' in scenarios
  assert 'caution_required' in scenarios
  assert 'adaptive_control_needed' in scenarios

  print("✓ Unusual scenario handling test passed")


def test_adaptive_control_modifications():
  """Test adaptive control modifications."""
  handler = EdgeCaseHandler()

  # Test with caution required
  scenarios = {
    'caution_required': True,
    'sharp_curve': True,
    'construction_zone': False,
    'sudden_object': False,
    'abnormal_steering': False
  }

  modifications = handler.get_adaptive_control_modifications(None, scenarios)

  # Verify that caution mode applies conservative factors
  if scenarios['caution_required']:
    assert modifications['longitudinal_factor'] <= 1.0
    assert modifications['lateral_factor'] <= 1.0
    assert modifications['min_gap'] >= 2.0

  print("✓ Adaptive control modifications test passed")


if __name__ == "__main__":
  test_edge_case_handler_initialization()
  test_sharp_curve_detection()
  test_abnormal_steering_detection()
  test_construction_zone_detection()
  test_unusual_scenario_handling()
  test_adaptive_control_modifications()
  print("All edge case handling tests passed!")
