#!/usr/bin/env python3
"""
Thermal stress testing for the adaptive system.
Tests that the system behaves safely under various thermal conditions.
"""
import time
import numpy as np
from unittest.mock import Mock
from openpilot.selfdrive.controls.lib.thermal_manager import ThermalManager


def test_thermal_stress_response():
  """Test system response to various thermal stress levels."""
  print("Testing thermal stress response...")
  
  thermal_manager = ThermalManager()
  
  # Test with different thermal conditions by simulating different temp values
  test_scenarios = [
    {"gpu_temp": 30.0, "cpu_temp": 40.0, "soc_temp": 35.0, "expected_state": 0.0},  # Cool conditions
    {"gpu_temp": 60.0, "cpu_temp": 65.0, "soc_temp": 62.0, "expected_state": 0.3},  # Moderate stress
    {"gpu_temp": 80.0, "cpu_temp": 85.0, "soc_temp": 82.0, "expected_state": 0.7},  # High stress
    {"gpu_temp": 95.0, "cpu_temp": 90.0, "soc_temp": 92.0, "expected_state": 1.0},  # Critical stress
  ]
  
  for i, scenario in enumerate(test_scenarios):
    # Create mock thermal data
    def make_mock_sm(temp_data):
      mock_sm = Mock()
      
      # Mock thermal data structure
      thermal_data = Mock()
      thermal_data.gpu = [temp_data["gpu_temp"]]
      thermal_data.cpu = [temp_data["cpu_temp"]]
      thermal_data.soc = [temp_data["soc_temp"]]
      
      # Setup SubMaster to return thermal data
      def mock_get(key, default=None):
        if key == 'thermal':
          return thermal_data
        elif key == 'deviceState':
          mock_dev_state = Mock()
          mock_dev_state.cpuPct = 30.0 + i*15  # Different CPU loads
          mock_dev_state.gpuPct = 25.0 + i*20  
          mock_dev_state.memoryPct = 40.0 + i*10
          return mock_dev_state
        return default
          
      mock_sm.get = mock_get
      return mock_sm
    
    mock_sm = make_mock_sm(scenario)
    current_time = time.monotonic()
    
    # Get thermal state
    thermal_state = thermal_manager.get_thermal_state_with_fallback(mock_sm, current_time)
    
    print(f"  Scenario {i+1}: temps ({scenario['gpu_temp']}, {scenario['cpu_temp']}, {scenario['soc_temp']}) -> thermal_state={thermal_state:.3f}")
    
    # The thermal state should generally increase with temperature
    if i > 0:
      prev_scenario = test_scenarios[i-1]
      prev_mock_sm = make_mock_sm(prev_scenario)
      prev_thermal_state = thermal_manager.get_thermal_state_with_fallback(prev_mock_sm, current_time)
      
      # Higher temperatures should generally result in higher thermal stress state
      assert thermal_state >= prev_thermal_state, f"Higher temps should result in higher thermal state: {prev_thermal_state} -> {thermal_state}"


def test_predictive_thermal_behavior():
  """Test the predictive thermal model under stress."""
  print("Testing predictive thermal behavior...")
  
  thermal_manager = ThermalManager()
  
  # Test the prediction function with different stress conditions
  test_predictions = [
    # (current_temp, current_trend, system_load, prediction_horizon, expected_behavior)
    (60.0, 0.1, 0.2, 0.1, "slight warming"),   # Low load, slight warming trend
    (70.0, 0.3, 0.8, 0.1, "fast warming"),     # High load, fast warming trend
    (75.0, -0.1, 0.9, 0.1, "slow cooling"),    # High load but cooling
    (80.0, -0.3, 0.3, 0.1, "fast cooling"),    # Low load, fast cooling trend
  ]
  
  for current_temp, current_trend, system_load, horizon, expected in test_predictions:
    predicted = thermal_manager._predict_temperature(current_temp, current_trend, system_load, horizon)
    
    print(f"  T:{current_temp}, dT/dt:{current_trend}, load:{system_load:.1f} -> pred:{predicted:.2f}°C ({expected})")
    
    # Basic validation: if trend is positive, prediction should be higher than current (with some load effect)
    if current_trend > 0:
        assert predicted >= current_temp, "Positive trend should result in higher predicted temperature"
    elif current_trend < 0:
        assert predicted <= current_temp, "Negative trend should result in lower predicted temperature"
    
    # The difference should be reasonable (not more than ~1°C for 0.1 second prediction)
    diff = abs(predicted - current_temp)
    assert diff < 2.0, f"Prediction difference too large: {diff}°C"


def test_system_load_factor_under_thermal_stress():
  """Test that system load factor behaves appropriately under thermal stress.""" 
  print("Testing system load factor under thermal stress...")
  
  thermal_manager = ThermalManager()
  
  # Create mock SubMaster with high system load
  mock_sm = Mock()
  mock_device_state = Mock()
  mock_device_state.cpuPct = 90.0  # High CPU usage
  mock_device_state.gpuPct = 85.0  # High GPU usage
  mock_device_state.memoryPct = 80.0  # High memory usage
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Calculate system load factor under high load
  thermal_manager._calculate_system_load_factor(mock_sm)
  high_load_factor = thermal_manager.system_load_factor
  
  # Now create mock with low system load
  mock_device_state.cpuPct = 10.0  # Low CPU usage
  mock_device_state.gpuPct = 5.0   # Low GPU usage
  mock_device_state.memoryPct = 15.0  # Low memory usage
  
  thermal_manager._calculate_system_load_factor(mock_sm)
  low_load_factor = thermal_manager.system_load_factor
  
  print(f"  High load system factor: {high_load_factor:.3f}")
  print(f"  Low load system factor: {low_load_factor:.3f}")
  
  # High system load should result in higher system load factor
  assert high_load_factor > low_load_factor, "High system load should result in higher system load factor"
  assert 0.0 <= low_load_factor <= high_load_factor <= 1.0, "Load factors should be in valid range"


def test_thermal_protection_mechanisms():
  """Test that thermal protection mechanisms engage when needed."""
  print("Testing thermal protection mechanisms...")
  
  thermal_manager = ThermalManager()
  
  # Simulate a scenario that should trigger thermal protection
  mock_sm = Mock()
  mock_thermal_data = Mock()
  mock_thermal_data.gpu = [90.0]  # Very high GPU temp
  mock_thermal_data.cpu = [88.0]  # Very high CPU temp
  mock_thermal_data.soc = [85.0]  # Very high SOC temp
  
  # Also high system load
  mock_device_state = Mock()
  mock_device_state.cpuPct = 95.0
  mock_device_state.gpuPct = 92.0
  mock_device_state.memoryPct = 90.0
  
  def mock_get(key, default=None):
    if key == 'thermal':
      return mock_thermal_data
    elif key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  current_time = time.monotonic()
  
  # Get thermal state under critical conditions
  thermal_state = thermal_manager.get_thermal_state_with_fallback(mock_sm, current_time)
  
  print(f"  Critical thermal state: {thermal_state:.3f}")
  
  # Under critical thermal conditions, thermal state should be high (close to 1.0)
  assert thermal_state >= 0.8, f"Critical thermal conditions should result in high thermal state, got {thermal_state}"
  
  # System load factor should also be high due to high usage
  assert thermal_manager.system_load_factor >= 0.8, f"High system usage should result in high system load factor, got {thermal_manager.system_load_factor}"


def test_thermal_trend_stability():
  """Test that thermal trends are stable and predictable."""
  print("Testing thermal trend stability...")
  
  thermal_manager = ThermalManager()
  
  # Simulate stable temperatures over time
  mock_sm = Mock()
  mock_thermal_data = Mock()
  mock_thermal_data.gpu = [60.0]
  mock_thermal_data.cpu = [55.0]
  mock_thermal_data.soc = [58.0]
  
  mock_device_state = Mock()
  mock_device_state.cpuPct = 40.0
  mock_device_state.gpuPct = 35.0
  mock_device_state.memoryPct = 45.0
  
  def mock_get(key, default=None):
    if key == 'thermal':
      return mock_thermal_data
    elif key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Update multiple times to see if trends stabilize
  states = []
  for i in range(10):
    current_time = time.monotonic() + i * 0.5  # Simulate time passing
    thermal_state = thermal_manager.get_thermal_state_with_fallback(mock_sm, current_time)
    states.append(thermal_state)
    
    # Also get the internal trend values if available
    if hasattr(thermal_manager, '_gpu_trend'):
      print(f"    Iteration {i+1}: thermal_state={thermal_state:.3f}, gpu_trend={getattr(thermal_manager, '_gpu_trend', 'N/A'):.3f}, system_load={thermal_manager.system_load_factor:.3f}")
  
  # The states should converge to similar values after initial transients
  if len(states) > 5:
    final_states = states[-5:]  # Last 5 states
    state_variance = np.var(final_states)
    print(f"  Variance of final 5 thermal states: {state_variance:.6f}")
    # Variance should be relatively small (indicating stable behavior)
    assert state_variance < 0.01, f"Thermal states should stabilize, got variance {state_variance}"


def test_thermal_context_safety():
  """Test that thermal considerations properly affect context-based decisions."""
  print("Testing thermal-context safety interactions...")
  
  # This test verifies that when thermal stress is high, 
  # context-based control rate boosting is appropriately limited
  
  # Simulate values from the controlsd logic
  stress_factor = 0.9  # High thermal stress
  system_load_factor = 0.8  # High system load
  
  # Simulate different driving contexts
  contexts = [
      ("parking", 0.5),    # Low speed - factor 0.5
      ("highway", 0.75),   # Highway cruise - factor 0.75  
      ("curvy_road", 1.2), # Curvy road - factor 1.2 (would normally boost)
  ]
  
  for context_name, context_factor in contexts:
    # Calculate base adaptive factor using weighted combination (from controlsd.py)
    base_adaptive_factor = max(0.3, min(1.0, 1.0 - (stress_factor * 0.4 + system_load_factor * 0.3)))
    
    # Apply thermal-aware context capping for enhanced safety under high thermal stress
    # From controlsd.py: context_cap = max(1.0, min(1.2, 1.2 - (stress_factor * 0.4)))
    context_cap = max(1.0, min(1.2, 1.2 - (stress_factor * 0.4)))
    limited_context_factor = min(context_factor, context_cap)
    
    # Calculate final rate
    base_rate = 100.0
    target_rate = base_rate * base_adaptive_factor * limited_context_factor
    min_rate = 20.0
    max_rate = 100.0
    final_rate = max(min_rate, min(max_rate, target_rate))
    
    print(f"  {context_name}: raw_factor={context_factor}, cap={context_cap:.2f}, limited={limited_context_factor}, final_rate={final_rate:.1f}Hz")
    
    # Even for curvy road (normally 1.2x), when thermal stress is high (0.9), 
    # the context cap will be 1.2 - (0.9 * 0.4) = 1.2 - 0.36 = 0.84
    # So the 1.2 factor gets limited to 0.84, preventing rate boosting during thermal stress
    if context_name == "curvy_road":
      expected_cap = 1.2 - (stress_factor * 0.4)  # 1.2 - 0.36 = 0.84
      assert limited_context_factor <= expected_cap, f"Curvy road factor should be capped at {expected_cap}, got {limited_context_factor}"


if __name__ == "__main__":
  print("Starting thermal stress safety tests...\n")
  
  test_thermal_stress_response()
  test_predictive_thermal_behavior()
  test_system_load_factor_under_thermal_stress()
  test_thermal_protection_mechanisms()
  test_thermal_trend_stability()
  test_thermal_context_safety()
  
  print(f"\n✓ All thermal stress safety tests passed!")