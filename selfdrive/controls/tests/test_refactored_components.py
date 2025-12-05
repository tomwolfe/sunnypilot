"""
Unit tests for the refactored control system components.
This ensures each module works correctly and maintains the original functionality.
"""

from unittest.mock import Mock, patch  # noqa: TID251

from openpilot.selfdrive.controls.lib.thermal_manager import ThermalManager
from openpilot.selfdrive.controls.lib.driving_context import DrivingContextAnalyzer
from openpilot.selfdrive.controls.lib.adaptive_gains_controller import AdaptiveGainsController
from openpilot.selfdrive.controls.lib.circuit_breaker_manager import CircuitBreakerManager


class TestThermalManager:
  """Test cases for the ThermalManager class."""

  def test_calculate_thermal_state(self):
    """Test thermal state calculation from device state."""
    thermal_manager = ThermalManager()

    # Create a mock device state with normal status (0)
    mock_device_state = Mock()
    mock_device_state.thermalStatus = 0  # Normal thermal status
    mock_device_state.thermalPerc = 75.0  # 75% thermal percentage

    # Test normal thermal calculation (thermalPerc / 100.0)
    thermal_state = thermal_manager.calculate_thermal_state(mock_device_state)
    expected_thermal = min(1.0, mock_device_state.thermalPerc / 100.0)
    assert thermal_state == expected_thermal

  def test_thermal_state_with_fallback(self):
    """Test thermal state calculation with stale data fallback."""
    thermal_manager = ThermalManager()

    # Create a mock submaster with valid device state
    mock_sm = {
      'deviceState': Mock(),
      'valid': {'deviceState': True}
    }
    mock_sm['deviceState'].thermalStatus = 0
    mock_sm['deviceState'].thermalPerc = 50.0

    current_time = 10.0
    thermal_state = thermal_manager.get_thermal_state_with_fallback(mock_sm, current_time)
    assert thermal_state == 0.5  # 50% thermal percentage = 0.5

  def test_gpu_management_not_on_non_android(self):
    """Test that GPU management is skipped on non-Android systems."""
    thermal_manager = ThermalManager()

    # Create mock parameters
    mock_sm = Mock()
    mock_CS = Mock()

    with patch('os.path.exists', return_value=False):
      # Should return early without errors
      thermal_manager.apply_gpu_management(mock_sm, mock_CS)


class TestDrivingContextAnalyzer:
  """Test cases for the DrivingContextAnalyzer class."""

  def test_calculate_driving_context(self):
    """Test driving context calculation."""
    context_analyzer = DrivingContextAnalyzer()

    # Create mock objects
    mock_CS = Mock()
    mock_CS.steeringAngleDeg = 0.0
    mock_CS.vEgo = 15.0
    mock_CS.aEgo = 0.0
    mock_CS.steeringRateDeg = 0.5

    mock_sm = {
      'liveParameters': Mock(),
      'deviceState': Mock(),
      'valid': {'deviceState': True}
    }
    mock_sm['liveParameters'].angleOffsetDeg = 0.0
    mock_sm['liveParameters'].roll = 0.0

    # Mock the VehicleModel
    mock_VM = Mock()
    mock_VM.calc_curvature.return_value = 0.001

    context = context_analyzer.calculate_driving_context(mock_CS, mock_sm, mock_VM)

    # Verify that context contains expected keys
    assert 'is_curvy_road' in context
    assert 'traffic_density' in context
    assert 'weather_condition' in context
    assert 'current_curvature' in context

  def test_detect_weather_conditions_with_wipers(self):
    """Test weather detection based on wiper status."""
    context_analyzer = DrivingContextAnalyzer()

    # Create mock state with wipers on
    mock_sm = {'carState': Mock()}
    mock_sm['carState'].windshieldWiper = 1.0  # Wipers on

    # Use internal method directly for testing
    weather = context_analyzer._detect_weather_conditions(mock_sm)
    assert weather == 'rain'

  def test_detect_weather_conditions_normal(self):
    """Test weather detection when no indicators are present."""
    context_analyzer = DrivingContextAnalyzer()

    # Create mock state with wipers off
    mock_sm = {'carState': Mock()}
    mock_sm['carState'].windshieldWiper = 0.0  # Wipers off
    mock_sm['carState'].wiperState = 0  # Alternative check

    weather = context_analyzer._detect_weather_conditions(mock_sm)
    assert weather == 'normal'


class TestAdaptiveGainsController:
  """Test cases for the AdaptiveGainsController class."""

  def test_calculate_contextual_adaptive_gains(self):
    """Test adaptive gain calculation."""
    gains_controller = AdaptiveGainsController()

    # Test with normal context
    context = {
      'is_curvy_road': False,
      'traffic_density': 'low',
      'weather_condition': 'normal',
      'current_curvature': 0.001,
      'lateral_accel': 1.0,
      'long_accel_magnitude': 1.0,
      'steering_activity': 0.5
    }

    adaptive_gains = gains_controller.calculate_contextual_adaptive_gains(15.0, 0.2, context)

    # Verify structure
    assert 'lateral' in adaptive_gains
    assert 'longitudinal' in adaptive_gains
    assert 'steer_kp' in adaptive_gains['lateral']
    assert 'accel_kp' in adaptive_gains['longitudinal']

  def test_validate_adaptive_gains_bounds(self):
    """Test gain validation within safe bounds."""
    gains_controller = AdaptiveGainsController()

    # Test with gains that should be clamped
    invalid_gains = {
      'lateral': {
        'steer_kp': 5.0,  # Too high, should be clamped to MAX_STEER_KP
        'steer_ki': -0.1,  # Too low, should be clamped
        'steer_kd': 0.2,  # Too high, should be clamped
      },
      'longitudinal': {
        'accel_kp': -0.5,  # Too low, should be clamped
        'accel_ki': 2.0,   # Too high, should be clamped
      },
    }

    validated_gains = gains_controller._validate_adaptive_gains(invalid_gains)

    # Verify gains are within bounds
    assert 0.1 <= validated_gains['lateral']['steer_kp'] <= 3.0
    assert 0.01 <= validated_gains['lateral']['steer_ki'] <= 1.0
    assert 0.0 <= validated_gains['lateral']['steer_kd'] <= 0.1
    assert 0.1 <= validated_gains['longitudinal']['accel_kp'] <= 2.0
    assert 0.01 <= validated_gains['longitudinal']['accel_ki'] <= 1.0


class TestCircuitBreakerManager:
  """Test cases for the CircuitBreakerManager class."""

  def test_initially_enabled(self):
    """Test that circuit breakers start enabled."""
    cb_manager = CircuitBreakerManager()

    assert cb_manager.check_circuit_breaker('adaptive_gains')
    assert cb_manager.check_circuit_breaker('radar_camera_fusion')
    assert cb_manager.check_circuit_breaker('vision_model_optimization')

  def test_trigger_circuit_breaker(self):
    """Test triggering a circuit breaker."""
    cb_manager = CircuitBreakerManager()

    # Initially should be enabled
    assert cb_manager.check_circuit_breaker('adaptive_gains')

    # Trigger the breaker
    cb_manager.trigger_circuit_breaker('adaptive_gains', 'Test error', 'test_error')

    # Should now be disabled
    assert not cb_manager.check_circuit_breaker('adaptive_gains')

  def test_circuit_breaker_reset_after_cooldown(self):
    """Test that circuit breakers reset after cooldown and stable period."""
    cb_manager = CircuitBreakerManager()

    # Manually set breaker state to test reset logic
    cb_manager.trigger_circuit_breaker('adaptive_gains', 'Test error', 'test_error')
    assert not cb_manager.check_circuit_breaker('adaptive_gains')

    # Set last error time in the past beyond both cooldown and stable period (10s + 5s = 15s total)
    import time
    current_time = time.monotonic()
    cb_manager._circuit_breakers['adaptive_gains']['last_error_time'] = current_time - 20.0  # 20s ago
    cb_manager._circuit_breakers['adaptive_gains']['last_error_reset_time'] = current_time - 20.0

    # Should now be enabled since both cooldown and stable period have passed
    assert cb_manager.check_circuit_breaker('adaptive_gains')


def run_tests():
  """Run all tests."""
  test_thermal = TestThermalManager()
  test_context = TestDrivingContextAnalyzer()
  test_gains = TestAdaptiveGainsController()
  test_circuit = TestCircuitBreakerManager()

  # Run thermal manager tests
  test_thermal.test_calculate_thermal_state()
  test_thermal.test_thermal_state_with_fallback()
  test_thermal.test_gpu_management_not_on_non_android()
  print("✓ ThermalManager tests passed")

  # Run context analyzer tests
  test_context.test_calculate_driving_context()
  test_context.test_detect_weather_conditions_with_wipers()
  test_context.test_detect_weather_conditions_normal()
  print("✓ DrivingContextAnalyzer tests passed")

  # Run adaptive gains tests
  test_gains.test_calculate_contextual_adaptive_gains()
  test_gains.test_validate_adaptive_gains_bounds()
  print("✓ AdaptiveGainsController tests passed")

  # Run circuit breaker tests
  test_circuit.test_initially_enabled()
  test_circuit.test_trigger_circuit_breaker()
  print("✓ CircuitBreakerManager tests passed")

  print("\nAll refactored component tests passed! ✓")


if __name__ == "__main__":
  run_tests()
