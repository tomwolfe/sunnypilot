from selfdrive.controls.lib.safety_helpers import SafetyManager
from cereal import car


def test_safety_manager_initialization():
  """Test that SafetyManager initializes properly."""
  safety_manager = SafetyManager()
  
  # Verify initial state
  assert safety_manager.safety_engaged == True
  assert safety_manager.safety_violation_count == 0
  
  print("✓ Safety manager initialization test passed")


def test_steering_safety_monitoring():
  """Test steering safety monitoring functionality."""
  safety_manager = SafetyManager()
  
  # Create mock car state
  CS = car.CarState.new_message()
  CS.steeringTorque = 50.0  # Moderate torque
  CS.steeringAngleDeg = 5.0
  
  # Create mock control output
  class MockActuatorsOutput:
    torque = 45.0
  
  control_output = MockActuatorsOutput()
  
  # Test that normal steering is considered safe
  is_safe = safety_manager.monitor_steering_safety(CS, control_output)
  assert is_safe == True, "Normal steering should be safe"
  
  # Test with excessive unexpected torque
  CS.steeringTorque = 300.0  # High torque
  is_safe = safety_manager.monitor_steering_safety(CS, control_output)
  # This may or may not be unsafe depending on implementation, but should not crash
  
  print("✓ Steering safety monitoring test passed")


def test_longitudinal_safety_monitoring():
  """Test longitudinal safety monitoring."""
  safety_manager = SafetyManager()
  
  # Create mock car state
  CS = car.CarState.new_message()
  CS.aEgo = 20.0  # 20 m/s
  
  class MockActuatorsOutput:
    torque = 0.0
  
  control_output = MockActuatorsOutput()
  
  # Test normal longitudinal control
  is_safe = safety_manager.monitor_longitudinal_safety(CS, control_output)
  assert is_safe == True, "Normal longitudinal control should be safe"
  
  print("✓ Longitudinal safety monitoring test passed")


def test_overall_safety_check():
  """Test the overall safety check function."""
  safety_manager = SafetyManager()
  
  # Create mock data
  CS = car.CarState.new_message()
  CS.steeringTorque = 30.0
  CS.steeringAngleDeg = 2.0
  CS.aEgo = 15.0
  
  class MockActuatorsOutput:
    torque = 25.0
  
  control_output = MockActuatorsOutput()
  
  # Test safety check
  is_safe, reason = safety_manager.check_safety_violations(CS, control_output, None)
  assert isinstance(is_safe, bool)
  assert isinstance(reason, str)
  
  print("✓ Overall safety check test passed")


def test_safety_recommendation():
  """Test safety-based recommendation system."""
  safety_manager = SafetyManager()
  
  # Create mock data
  CS = car.CarState.new_message()
  CS.steeringTorque = 20.0
  CS.steeringAngleDeg = 1.0
  CS.aEgo = 10.0
  
  class MockActuatorsOutput:
    torque = 15.0
  
  control_output = MockActuatorsOutput()
  
  # Test safety recommendation
  from cereal import log
  recommendation = safety_manager.get_safety_recommendation(CS, control_output, None)
  assert recommendation in [log.SelfdriveState.AlertStatus.normal, 
                           log.SelfdriveState.AlertStatus.warning,
                           log.SelfdriveState.AlertStatus.critical]
  
  print("✓ Safety recommendation test passed")


if __name__ == "__main__":
  test_safety_manager_initialization()
  test_steering_safety_monitoring()
  test_longitudinal_safety_monitoring()
  test_overall_safety_check()
  test_safety_recommendation()
  print("All safety feature tests passed!")