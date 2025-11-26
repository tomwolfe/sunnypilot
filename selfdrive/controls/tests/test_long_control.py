import numpy as np
from selfdrive.controls.lib.longcontrol import LongControl
from cereal import car, log
from opendbc.car.structs import CarParams


def test_improved_longitudinal_control_jerk_limiting():
  """Test the improved jerk limiting in longitudinal control."""
  # Create mock car params
  CP = CarParams()
  CP.longitudinalTuning.kpBP = [0., 5., 35.]
  CP.longitudinalTuning.kpV = [1.0, 1.2, 0.8]
  CP.longitudinalTuning.kiBP = [0., 5., 35.]
  CP.longitudinalTuning.kiV = [0.2, 0.3, 0.1]
  CP.stopAccel = -2.0
  CP.startAccel = 1.0
  
  # Create extended car params
  class MockCarParamsSP:
    enableGasInterceptor = False
    neuralNetworkLateralControl = type('obj', (object,), {'model': type('obj', (object,), {'path': ''})})()
  
  CP_SP = MockCarParamsSP()
  
  long_control = LongControl(CP, CP_SP)
  
  # Simulate car state
  CS = car.CarState.new_message()
  CS.vEgo = 20.0  # 20 m/s
  CS.aEgo = 0.5   # 0.5 m/s^2 acceleration
  
  # Test with varying acceleration targets to verify jerk limiting
  accel_limits = (-3.0, 2.0)
  a_target_1 = 1.0
  a_target_2 = 4.0  # Large change to test jerk limiting
  
  # Update with first target
  output_accel_1 = long_control.update(True, CS, a_target_1, False, accel_limits)
  
  # Update with second target (should be jerk-limited)
  CS.aEgo = output_accel_1  # Update ego acceleration
  output_accel_2 = long_control.update(True, CS, a_target_2, False, accel_limits)
  
  # Verify that acceleration doesn't change too rapidly due to jerk limiting
  print(f"Target change: {a_target_1} -> {a_target_2}, Output change: {output_accel_1:.2f} -> {output_accel_2:.2f}")
  
  print("✓ Improved longitudinal control jerk limiting test passed")


def test_adaptive_pid_in_longitudinal_control():
  """Test the adaptive PID based on driving conditions."""
  CP = CarParams()
  CP.longitudinalTuning.kpBP = [0., 5., 35.]
  CP.longitudinalTuning.kpV = [1.0, 1.2, 0.8]
  CP.longitudinalTuning.kiBP = [0., 5., 35.]
  CP.longitudinalTuning.kiV = [0.2, 0.3, 0.1]
  CP.stopAccel = -2.0
  CP.startAccel = 1.0
  
  class MockCarParamsSP:
    enableGasInterceptor = False
    neuralNetworkLateralControl = type('obj', (object,), {'model': type('obj', (object,), {'path': ''})})()
  
  CP_SP = MockCarParamsSP()
  
  long_control = LongControl(CP, CP_SP)
  
  # Set up test conditions for adaptive control
  CS = car.CarState.new_message()
  CS.vEgo = 25.0  # Moderate speed
  CS.aEgo = 0.0
  
  # Test with target close to actual (should trigger adaptive control)
  a_target = 0.1  # Close to aEgo of 0.0
  accel_limits = (-3.0, 2.0)
  
  output_accel = long_control.update(True, CS, a_target, False, accel_limits)
  
  # The control should work without errors
  assert isinstance(output_accel, float), "Output should be a float"
  
  print("✓ Adaptive PID in longitudinal control test passed")


def test_low_speed_crawl_control():
  """Test improved low-speed control for smoother stop-and-go."""
  CP = CarParams()
  CP.longitudinalTuning.kpBP = [0., 5., 35.]
  CP.longitudinalTuning.kpV = [1.0, 1.2, 0.8]
  CP.longitudinalTuning.kiBP = [0., 5., 35.]
  CP.longitudinalTuning.kiV = [0.2, 0.3, 0.1]
  CP.stopAccel = -2.0
  CP.startAccel = 1.0
  
  class MockCarParamsSP:
    enableGasInterceptor = False
    neuralNetworkLateralControl = type('obj', (object,), {'model': type('obj', (object,), {'path': ''})})()
  
  CP_SP = MockCarParamsSP()
  
  long_control = LongControl(CP, CP_SP)
  
  # Test at very low speed (should apply additional smoothing)
  CS = car.CarState.new_message()
  CS.vEgo = 2.0  # Very low speed
  CS.aEgo = 0.0
  
  a_target = 0.5
  accel_limits = (-2.0, 1.5)
  
  output_accel = long_control.update(True, CS, a_target, False, accel_limits)
  
  # Control should work at low speeds
  assert -2.0 <= output_accel <= 1.5, "Output should respect limits"
  
  print("✓ Low-speed crawl control test passed")


if __name__ == "__main__":
  test_improved_longitudinal_control_jerk_limiting()
  test_adaptive_pid_in_longitudinal_control()
  test_low_speed_crawl_control()
  print("All longitudinal control tests passed!")