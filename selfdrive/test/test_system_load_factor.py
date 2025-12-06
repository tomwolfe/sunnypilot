#!/usr/bin/env python3
"""
Unit tests for the new system load factor calculation in thermal manager.
This test verifies the _calculate_system_load_factor method and related functionality.
"""
import time
import numpy as np
from unittest.mock import Mock, MagicMock
from openpilot.selfdrive.controls.lib.thermal_manager import ThermalManager


def test_system_load_factor_calculation():
  """Test the system load factor calculation with various inputs."""
  print("Testing system load factor calculation...")

  thermal_manager = ThermalManager()

  # Create a mock SubMaster with device state data
  mock_sm = Mock()

  # Set up the get method to return the device state only for 'deviceState' key
  mock_device_state = Mock()
  mock_device_state.cpuPct = 50.0  # 50% CPU usage
  mock_device_state.gpuPct = 30.0  # 30% GPU usage
  mock_device_state.memoryPct = 70.0  # 70% memory usage

  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default

  mock_sm.get = mock_get

  # Call the system load calculation method
  thermal_manager._calculate_system_load_factor(mock_sm)

  # With EMA, first call with avg (0.5+0.3+0.7)/3=0.5, result = 0.3 * 0.5 + 0.7 * 0.0 = 0.15
  expected_value = 0.3 * (0.5 + 0.3 + 0.7) / 3  # = 0.15
  assert abs(thermal_manager.system_load_factor - expected_value) < 0.01, f"Expected ~{expected_value:.3f}, got {thermal_manager.system_load_factor:.3f}"
  assert hasattr(thermal_manager, 'system_load_factor'), "System load factor attribute should exist"
  assert 0.0 <= thermal_manager.system_load_factor <= 1.0, "System load factor should be between 0.0 and 1.0"

  print(f"✓ System load factor calculated: {thermal_manager.system_load_factor:.3f}")
  

def test_system_load_factor_with_high_usage():
  """Test system load factor with high usage values."""
  print("Testing system load factor with high usage...")

  thermal_manager = ThermalManager()

  # Create a mock SubMaster with high device state data
  mock_sm = Mock()

  # Set up the get method to return the device state only for 'deviceState' key
  mock_device_state = Mock()
  mock_device_state.cpuPct = 90.0  # 90% CPU usage
  mock_device_state.gpuPct = 80.0  # 80% GPU usage
  mock_device_state.memoryPct = 95.0  # 95% memory usage

  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default

  mock_sm.get = mock_get

  # Call the system load calculation method
  thermal_manager._calculate_system_load_factor(mock_sm)

  # With exponential moving average, the first call uses alpha=0.3, previous=0.0, current_avg=(0.9+0.8+0.95)/3=0.883
  # So result = 0.3 * 0.883 + 0.7 * 0.0 ≈ 0.265
  expected_first_value = 0.3 * (0.9 + 0.8 + 0.95) / 3  # ≈ 0.265
  assert abs(thermal_manager.system_load_factor - expected_first_value) < 0.01, f"Expected ~{expected_first_value:.3f}, got {thermal_manager.system_load_factor:.3f}"

  print(f"✓ High usage system load factor (first call, with EMA): {thermal_manager.system_load_factor:.3f}")

  # Call again to see EMA behavior - should get closer to the true value
  thermal_manager._calculate_system_load_factor(mock_sm)
  # Second call: alpha=0.3, previous=0.265, current_avg=0.883
  # result = 0.3 * 0.883 + 0.7 * 0.265 ≈ 0.265 + 0.186 ≈ 0.451
  print(f"✓ High usage system load factor (second call): {thermal_manager.system_load_factor:.3f}")

  # Make multiple calls to get close to steady state
  for i in range(10):
    thermal_manager._calculate_system_load_factor(mock_sm)

  # After multiple calls, should be very close to the average
  expected_avg = (0.90 + 0.80 + 0.95) / 3  # 0.883
  assert abs(thermal_manager.system_load_factor - expected_avg) < 0.05, f"Expected ~{expected_avg:.3f}, got {thermal_manager.system_load_factor:.3f}"
  print(f"✓ High usage system load factor (converged): {thermal_manager.system_load_factor:.3f}")


def test_system_load_factor_with_low_usage():
  """Test system load factor with low usage values."""
  print("Testing system load factor with low usage...")

  thermal_manager = ThermalManager()

  # Create a mock SubMaster with low device state data
  mock_sm = Mock()

  # Set up the get method to return the device state only for 'deviceState' key
  mock_device_state = Mock()
  mock_device_state.cpuPct = 10.0  # 10% CPU usage
  mock_device_state.gpuPct = 5.0  # 5% GPU usage
  mock_device_state.memoryPct = 20.0  # 20% memory usage

  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default

  mock_sm.get = mock_get

  # Call the system load calculation method
  thermal_manager._calculate_system_load_factor(mock_sm)

  # With EMA, first call with avg (0.1+0.05+0.2)/3=0.117, result = 0.3 * 0.117 + 0.7 * 0.0 = 0.035
  expected_value = 0.3 * (0.1 + 0.05 + 0.2) / 3  # = 0.035
  assert abs(thermal_manager.system_load_factor - expected_value) < 0.01, f"Expected ~{expected_value:.3f}, got {thermal_manager.system_load_factor:.3f}"

  print(f"✓ Low usage system load factor: {thermal_manager.system_load_factor:.3f}")


def test_system_load_factor_missing_data():
  """Test system load factor when some usage data is missing."""
  print("Testing system load factor with missing data...")

  thermal_manager = ThermalManager()

  # Create a mock SubMaster with some missing device state data
  mock_sm = Mock()

  # Set up the get method to return the device state only for 'deviceState' key
  mock_device_state = Mock()
  mock_device_state.cpuPct = 60.0  # 60% CPU usage
  mock_device_state.gpuPct = None  # Missing GPU usage
  mock_device_state.memoryPct = 40.0  # 40% memory usage

  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default

  mock_sm.get = mock_get

  # Call the system load calculation method
  thermal_manager._calculate_system_load_factor(mock_sm)

  # With CPU=60% and memory=40% (GPU missing), avg = (0.6+0.4)/2 = 0.5
  # With EMA: result = 0.3 * 0.5 + 0.7 * 0.0 = 0.15
  expected_value = 0.3 * (0.6 + 0.4) / 2  # = 0.15
  assert abs(thermal_manager.system_load_factor - expected_value) < 0.01, f"Expected ~{expected_value:.3f}, got {thermal_manager.system_load_factor:.3f}"
  assert 0.0 <= thermal_manager.system_load_factor <= 1.0, "System load factor should be valid with partial data"

  print(f"✓ Missing data system load factor: {thermal_manager.system_load_factor:.3f}")


def test_predict_temperature_function():
  """Test the temperature prediction function."""
  print("Testing temperature prediction function...")
  
  thermal_manager = ThermalManager()
  
  # Test with no load (system_load_factor = 0.0)
  current_temp = 60.0
  current_trend = 0.5  # Rising at 0.5°C per second
  system_load_factor = 0.0  # No load
  prediction_horizon = 0.1  # 100ms ahead
  
  predicted_temp = thermal_manager._predict_temperature(current_temp, current_trend, system_load_factor, prediction_horizon)
  
  # Should be close to: 60.0 + (0.5 * 0.1) = 60.05
  expected_temp = current_temp + (current_trend * prediction_horizon)
  assert abs(predicted_temp - expected_temp) < 0.1, f"Expected ~{expected_temp}, got {predicted_temp}"
  
  print(f"✓ Temperature prediction (no load): {current_temp} -> {predicted_temp:.2f}")
  
  # Test with high load (system_load_factor = 1.0)
  system_load_factor = 1.0  # High load
  predicted_temp_high_load = thermal_manager._predict_temperature(current_temp, current_trend, system_load_factor, prediction_horizon)
  
  # With high load, temperature should rise faster
  assert predicted_temp_high_load > predicted_temp, "Higher load should result in higher predicted temperature"
  
  print(f"✓ Temperature prediction (high load): {current_temp} -> {predicted_temp_high_load:.2f}")


def test_predict_temperature_cooling():
  """Test the temperature prediction when cooling."""
  print("Testing temperature prediction during cooling...")
  
  thermal_manager = ThermalManager()
  
  # Test with cooling trend (negative)
  current_temp = 60.0
  current_trend = -0.3  # Cooling at 0.3°C per second
  system_load_factor = 0.8  # High load while cooling
  prediction_horizon = 0.1  # 100ms ahead
  
  predicted_temp = thermal_manager._predict_temperature(current_temp, current_trend, system_load_factor, prediction_horizon)
  
  # Should still be cooling but at a reduced rate due to load effect
  # Base cooling: 60.0 + (-0.3 * 0.1) = 59.97
  expected_base = current_temp + (current_trend * prediction_horizon)
  
  assert predicted_temp < current_temp, "Should still be cooling with negative trend"
  assert predicted_temp >= expected_base, "Load factor should slow down cooling"
  
  print(f"✓ Cooling prediction: {current_temp} -> {predicted_temp:.2f} (base: {expected_base:.2f})")


def test_exponential_moving_average_load():
  """Test that the exponential moving average properly smooths load values."""
  print("Testing exponential moving average for load smoothing...")

  thermal_manager = ThermalManager()

  # Create mock SubMaster
  mock_sm = Mock()

  # Set up the get method to return the device state only for 'deviceState' key
  mock_device_state = Mock()

  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default

  mock_sm.get = mock_get

  # Simulate high load value
  mock_device_state.cpuPct = 90.0
  mock_device_state.gpuPct = 80.0
  mock_device_state.memoryPct = 85.0

  thermal_manager._calculate_system_load_factor(mock_sm)
  first_load = thermal_manager.system_load_factor

  print(f"  First call with high load: {first_load:.3f}")

  # Now simulate lower load value
  mock_device_state.cpuPct = 30.0
  mock_device_state.gpuPct = 20.0
  mock_device_state.memoryPct = 25.0

  thermal_manager._calculate_system_load_factor(mock_sm)
  second_load = thermal_manager.system_load_factor

  print(f"  Second call with low load: {second_load:.3f}")

  # The second value should be between the first high value and the current steady state
  # With EMA: new_avg = (0.3+0.2+0.25)/3 = 0.25, prev=first_load, result=0.3*0.25 + 0.7*first_load
  expected_second = 0.3 * ((0.3 + 0.2 + 0.25) / 3) + 0.7 * first_load
  assert abs(second_load - expected_second) < 0.01, f"Expected ~{expected_second:.3f}, got {second_load:.3f}"

  # The second value should still be higher than the raw average of the second call due to smoothing
  raw_avg_second = (0.3 + 0.2 + 0.25) / 3  # 0.25
  assert second_load > raw_avg_second, f"EMA should be above raw average {raw_avg_second:.3f} due to history"

  assert second_load < first_load, "Load factor should decrease after lower usage"
  # But not all the way to the instantaneous low value due to smoothing
  print(f"✓ EMA smoothing: High load ({first_load:.3f}) -> Lower load but smoothed to ({second_load:.3f})")


if __name__ == "__main__":
  test_system_load_factor_calculation()
  test_system_load_factor_with_high_usage()
  test_system_load_factor_with_low_usage()
  test_system_load_factor_missing_data()
  test_predict_temperature_function()
  test_predict_temperature_cooling()
  test_exponential_moving_average_load()
  print("All system load factor tests passed!")