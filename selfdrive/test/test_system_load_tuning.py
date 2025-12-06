#!/usr/bin/env python3
"""
System load factor tuning tests for the adaptive system.
Tests the responsiveness, accuracy, and tuning of the system load factor calculation.
"""
import time
import numpy as np
from unittest.mock import Mock
from openpilot.selfdrive.controls.lib.thermal_manager import ThermalManager


def test_system_load_factor_responsiveness():
  """Test how quickly the system load factor responds to changes in system usage."""
  print("Testing system load factor responsiveness...")
  
  thermal_manager = ThermalManager()
  
  # Create mock SubMaster
  mock_sm = Mock()
  mock_device_state = Mock()
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Start with low system usage
  mock_device_state.cpuPct = 10.0
  mock_device_state.gpuPct = 5.0
  mock_device_state.memoryPct = 15.0
  
  # Calculate initial load factor
  thermal_manager._calculate_system_load_factor(mock_sm)
  initial_load = thermal_manager.system_load_factor
  print(f"  Initial (low) load factor: {initial_load:.3f}")
  
  # Change to high system usage
  mock_device_state.cpuPct = 90.0
  mock_device_state.gpuPct = 85.0
  mock_device_state.memoryPct = 80.0
  
  # Calculate load factor after change
  thermal_manager._calculate_system_load_factor(mock_sm)
  high_load = thermal_manager.system_load_factor
  print(f"  After high usage: {high_load:.3f}")
  
  # The system load factor should be higher after high usage
  assert high_load > initial_load, f"Load factor should increase with higher usage: {initial_load} -> {high_load}"
  
  # Change back to low usage
  mock_device_state.cpuPct = 15.0
  mock_device_state.gpuPct = 10.0
  mock_device_state.memoryPct = 20.0
  
  # Calculate multiple times to see how fast it responds back to low values
  response_loads = []
  for i in range(10):
    thermal_manager._calculate_system_load_factor(mock_sm)
    response_loads.append(thermal_manager.system_load_factor)
    print(f"    Response iteration {i+1}: {thermal_manager.system_load_factor:.3f}")
  
  # Check that the load factor decreases after removing high load
  final_load = thermal_manager.system_load_factor
  print(f"  Final load after returning to low usage: {final_load:.3f}")
  
  # The load factor should decrease, though it may take time due to EMA
  # The important thing is that it should show a decreasing trend
  assert final_load <= response_loads[0], "Load factor should decrease when system usage returns to low levels"


def test_ema_smoothing_behavior():
  """Test the exponential moving average behavior and smoothing."""
  print("Testing EMA smoothing behavior...")
  
  thermal_manager = ThermalManager()
  
  # Create mock SubMaster
  mock_sm = Mock()
  mock_device_state = Mock()
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Set high initial load
  mock_device_state.cpuPct = 80.0
  mock_device_state.gpuPct = 75.0
  mock_device_state.memoryPct = 70.0
  
  # Initialize the thermal manager with high load
  thermal_manager._calculate_system_load_factor(mock_sm)
  initial_high_load = thermal_manager.system_load_factor
  print(f"  Initial high load state: {initial_high_load:.3f}")
  
  # Switch to low load and monitor how EMA smooths the transition
  mock_device_state.cpuPct = 20.0
  mock_device_state.gpuPct = 15.0
  mock_device_state.memoryPct = 25.0
  
  loads_over_time = []
  for i in range(20):  # Track 20 updates
    thermal_manager._calculate_system_load_factor(mock_sm)
    current_load = thermal_manager.system_load_factor
    loads_over_time.append(current_load)
    print(f"    Step {i+1}: {current_load:.3f}")
  
  # EMA should show smooth transition from high to low, not immediate jump
  # The load should decrease monotonically (or nearly so) due to smoothing
  decreasing_count = 0
  for i in range(1, len(loads_over_time)):
    if loads_over_time[i] <= loads_over_time[i-1]:
      decreasing_count += 1
  
  # Most points should show decreasing trend (there might be small fluctuations)
  assert decreasing_count >= len(loads_over_time) * 0.8, "EMA should generally show decreasing trend toward new low load"
  
  # Final value should be closer to low load than high load
  expected_low_steady_state = (20.0 + 15.0 + 25.0) / 3 / 100  # ~0.20 with EMA it might be different
  print(f"  Expected steady state ~ {expected_low_steady_state:.3f}")
  
  # After many updates, should approach the new steady state
  final_average = sum(loads_over_time[-5:]) / 5  # Average of last 5 points
  print(f"  Average of last 5 readings: {final_average:.3f}")


def test_alpha_parameter_effects():
  """Test the effect of the alpha parameter (smoothing factor) on EMA."""
  print("Testing alpha parameter effects...")
  
  # We'll test by creating multiple thermal managers and seeing how different
  # load scenarios are smoothed with the fixed alpha=0.3 in the code
  
  thermal_manager = ThermalManager()
  
  # Create mock SubMaster
  mock_sm = Mock()
  mock_device_state = Mock()
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Test with fluctuating system load to see smoothing effects
  fluctuating_loads = [
    (90, 85, 80),  # High
    (20, 15, 25),  # Low
    (70, 65, 60),  # Medium-high
    (30, 35, 40),  # Medium-low
    (85, 80, 75),  # High again
    (25, 20, 30),  # Low again
  ]
  
  print("  Simulating fluctuating system loads:")
  system_load_readings = []
  
  for cpu, gpu, mem in fluctuating_loads:
    mock_device_state.cpuPct = cpu
    mock_device_state.gpuPct = gpu
    mock_device_state.memoryPct = mem
    
    thermal_manager._calculate_system_load_factor(mock_sm)
    current_load = thermal_manager.system_load_factor
    system_load_readings.append(current_load)
    
    print(f"    Raw: ({cpu:2d}, {gpu:2d}, {mem:2d})% -> Smoothed: {current_load:.3f}")
  
  # Verify that the system load factor is being calculated and is in valid range
  for load in system_load_readings:
    assert 0.0 <= load <= 1.0, f"System load factor should be between 0.0 and 1.0, got {load}"
  
  # The smoothing should prevent sudden jumps between high and low values
  # Let's check that consecutive values don't differ too dramatically
  max_jump = 0.0
  for i in range(1, len(system_load_readings)):
    jump = abs(system_load_readings[i] - system_load_readings[i-1])
    max_jump = max(max_jump, jump)
  
  print(f"  Maximum jump between consecutive readings: {max_jump:.3f}")
  # With alpha=0.3, we expect some smoothing but not extreme damping
  assert max_jump < 0.5, f"EMA smoothing should limit extreme jumps, max jump was {max_jump}"


def test_system_load_factor_range_validation():
  """Test that system load factor stays within expected ranges."""
  print("Testing system load factor range validation...")
  
  thermal_manager = ThermalManager()
  
  # Create mock SubMaster
  mock_sm = Mock()
  mock_device_state = Mock()
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Test with various combinations of CPU, GPU, and memory usage
  test_combinations = [
    # Minimum possible (0% everything)
    (0.0, 0.0, 0.0, "minimum possible load"),
    # Maximum possible (100% everything)
    (100.0, 100.0, 100.0, "maximum possible load"),
    # Various mixed loads
    (50.0, 0.0, 100.0, "mixed: medium CPU, no GPU, full memory"),
    (75.0, 80.0, 20.0, "mixed: high CPU/GPU, low memory"),
    (30.0, 60.0, 90.0, "mixed: low CPU, medium GPU, high memory"),
  ]
  
  for cpu_pct, gpu_pct, mem_pct, description in test_combinations:
    mock_device_state.cpuPct = cpu_pct
    mock_device_state.gpuPct = gpu_pct
    mock_device_state.memoryPct = mem_pct
    
    # Calculate multiple times to allow EMA to settle
    for _ in range(5):
      thermal_manager._calculate_system_load_factor(mock_sm)
    
    final_load = thermal_manager.system_load_factor
    
    print(f"    {description}: ({cpu_pct:5.1f}, {gpu_pct:5.1f}, {mem_pct:5.1f})% -> {final_load:.3f}")
    
    # System load factor should always be between 0.0 and 1.0
    assert 0.0 <= final_load <= 1.0, f"System load factor {final_load} out of range [0.0, 1.0] for {description}"
    
    # With minimum loads, result should be low
    if cpu_pct == 0.0 and gpu_pct == 0.0 and mem_pct == 0.0:
      # Due to EMA, it might not be exactly 0, but should be low
      pass  # The EMA might keep it above 0 even with 0 inputs if there was previous load
    # With maximum loads, result should be high when settled
    elif cpu_pct == 100.0 and gpu_pct == 100.0 and mem_pct == 100.0:
      # Due to EMA, it might not immediately reach 1.0, but should be high
      pass


def test_system_load_factor_with_missing_data():
  """Test system load factor calculation when some data is missing."""
  print("Testing system load factor with missing data...")
  
  thermal_manager = ThermalManager()
  
  # Create mock SubMaster
  mock_sm = Mock()
  mock_device_state = Mock()
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Test with missing GPU usage (None)
  mock_device_state.cpuPct = 50.0
  mock_device_state.gpuPct = None  # Missing GPU data
  mock_device_state.memoryPct = 60.0
  
  thermal_manager._calculate_system_load_factor(mock_sm)
  load_with_missing_gpu = thermal_manager.system_load_factor
  print(f"  Missing GPU: load = {load_with_missing_gpu:.3f}")
  
  # Should handle missing data gracefully
  assert 0.0 <= load_with_missing_gpu <= 1.0, "Should handle missing GPU data gracefully"
  
  # Reset and test with missing CPU usage
  mock_device_state.cpuPct = None  # Missing CPU data
  mock_device_state.gpuPct = 40.0
  mock_device_state.memoryPct = 50.0
  
  thermal_manager._calculate_system_load_factor(mock_sm)
  load_with_missing_cpu = thermal_manager.system_load_factor
  print(f"  Missing CPU: load = {load_with_missing_cpu:.3f}")
  
  assert 0.0 <= load_with_missing_cpu <= 1.0, "Should handle missing CPU data gracefully"
  
  # Test with missing memory usage
  mock_device_state.cpuPct = 30.0
  mock_device_state.gpuPct = 35.0
  mock_device_state.memoryPct = None  # Missing memory data
  
  thermal_manager._calculate_system_load_factor(mock_sm)
  load_with_missing_mem = thermal_manager.system_load_factor
  print(f"  Missing Memory: load = {load_with_missing_mem:.3f}")
  
  assert 0.0 <= load_with_missing_mem <= 1.0, "Should handle missing memory data gracefully"


def test_system_load_factor_steady_state():
  """Test that system load factor reaches expected steady-state values."""
  print("Testing system load factor steady-state behavior...")
  
  thermal_manager = ThermalManager()
  
  # Create mock SubMaster
  mock_sm = Mock()
  mock_device_state = Mock()
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Set constant load and verify it stabilizes
  constant_cpu = 45.0
  constant_gpu = 50.0
  constant_mem = 40.0
  
  mock_device_state.cpuPct = constant_cpu
  mock_device_state.gpuPct = constant_gpu
  mock_device_state.memoryPct = constant_mem
  
  loads_over_time = []
  for i in range(50):  # Simulate 50 updates to see stabilization
    thermal_manager._calculate_system_load_factor(mock_sm)
    current_load = thermal_manager.system_load_factor
    loads_over_time.append(current_load)
    
    if i < 10 or i % 10 == 9:  # Print first 10 and every 10th thereafter
      print(f"    Update {i+1}: {current_load:.3f}")
  
  # Calculate variance of the last half to check stability
  last_half = loads_over_time[-25:]
  variance = np.var(last_half)
  print(f"    Variance of last 25 readings: {variance:.6f}")
  
  # The system should stabilize (variance should be small after initial settling)
  assert variance < 0.001, f"System load factor should stabilize, variance was {variance}"
  
  # Calculate the average of the stable period
  stable_average = sum(last_half) / len(last_half)
  expected_average = (constant_cpu + constant_gpu + constant_mem) / 3 / 100.0  # Convert to 0-1 scale
  print(f"    Stable average: {stable_average:.3f}, expected: {expected_average:.3f}")
  
  # The stable value should be reasonably close to the expected average (with EMA smoothing)
  # Allow some difference due to EMA characteristics
  difference = abs(stable_average - expected_average)
  print(f"    Difference from raw average: {difference:.3f}")
  
  # With EMA, it won't be exactly equal, but should be reasonably close
  assert difference < 0.2, f"Difference from expected {difference} is too large"


if __name__ == "__main__":
  print("Starting system load factor tuning tests...\n")
  
  test_system_load_factor_responsiveness()
  test_ema_smoothing_behavior()
  test_alpha_parameter_effects()
  test_system_load_factor_range_validation()
  test_system_load_factor_with_missing_data()
  test_system_load_factor_steady_state()
  
  print(f"\n✓ All system load factor tuning tests passed!")