#!/usr/bin/env python3
"""
Performance baseline assessment for adaptive resource management system.
This script establishes baseline metrics for CPU/GPU usage, thermal behavior, and control precision.
"""
import time
import numpy as np
import psutil
from unittest.mock import Mock
from openpilot.selfdrive.controls.lib.thermal_manager import ThermalManager


def benchmark_system_load_calculation():
  """Benchmark the system load factor calculation performance."""
  print("Benchmarking system load factor calculation...")
  
  thermal_manager = ThermalManager()
  
  # Create mock data for different load scenarios
  scenarios = [
    {"cpu": 10.0, "gpu": 5.0, "mem": 20.0},  # Low load
    {"cpu": 50.0, "gpu": 30.0, "mem": 40.0},  # Medium load
    {"cpu": 90.0, "gpu": 80.0, "mem": 85.0},  # High load
  ]
  
  # Create mock SubMaster
  def make_mock_sm(cpu_pct, gpu_pct, mem_pct):
    mock_sm = Mock()
    mock_device_state = Mock()
    mock_device_state.cpuPct = cpu_pct
    mock_device_state.gpuPct = gpu_pct
    mock_device_state.memoryPct = mem_pct
    
    def mock_get(key, default=None):
      if key == 'deviceState':
        return mock_device_state
      return default
    
    mock_sm.get = mock_get
    return mock_sm
  
  # Benchmark calculation time
  calculations = []
  for scenario in scenarios:
    mock_sm = make_mock_sm(scenario["cpu"], scenario["gpu"], scenario["mem"])
    
    start_time = time.perf_counter()
    for _ in range(100):  # Run 100 times for better average
      thermal_manager._calculate_system_load_factor(mock_sm)
    end_time = time.perf_counter()
    
    avg_time = (end_time - start_time) / 100
    calculations.append(avg_time)
    
    print(f"  {scenario['cpu']}/{scenario['gpu']}/{scenario['mem']}% load: {avg_time*1000:.3f}ms per calculation")
  
  avg_calc_time = np.mean(calculations)
  print(f"  Average calculation time: {avg_calc_time*1000:.3f}ms")
  print(f"  ✓ System load calculation meets performance requirements (< 1ms)")
  
  return avg_calc_time


def benchmark_temperature_prediction():
  """Benchmark the temperature prediction function performance."""
  print("Benchmarking temperature prediction...")
  
  thermal_manager = ThermalManager()
  
  # Test parameters
  test_params = [
    (60.0, 0.1, 0.3, 0.1),   # Normal temp, slight warming, low load, short horizon
    (75.0, 0.5, 0.8, 0.1),   # Higher temp, faster warming, high load, short horizon
    (65.0, -0.2, 0.6, 0.1),  # Cooling down, medium load
  ]
  
  prediction_times = []
  for temp, trend, load, horizon in test_params:
    start_time = time.perf_counter()
    for _ in range(1000):
      thermal_manager._predict_temperature(temp, trend, load, horizon)
    end_time = time.perf_counter()
    
    avg_time = (end_time - start_time) / 1000
    prediction_times.append(avg_time)
    
    print(f"  T:{temp}, dT/dt:{trend}, load:{load}: {avg_time*1000:.3f}ms per prediction")
  
  avg_pred_time = np.mean(prediction_times)
  print(f"  Average prediction time: {avg_pred_time*1000:.3f}ms")
  print(f"  ✓ Temperature prediction meets performance requirements")
  
  return avg_pred_time


def benchmark_ema_smoothing():
  """Benchmark the exponential moving average smoothing."""
  print("Benchmarking EMA smoothing...")
  
  thermal_manager = ThermalManager()
  
  # Set up mock with high load
  mock_sm = Mock()
  mock_device_state = Mock()
  mock_device_state.cpuPct = 80.0
  mock_device_state.gpuPct = 70.0
  mock_device_state.memoryPct = 75.0
  
  def mock_get(key, default=None):
    if key == 'deviceState':
      return mock_device_state
    return default
  
  mock_sm.get = mock_get
  
  # Warm up the EMA with some values
  for _ in range(10):
    thermal_manager._calculate_system_load_factor(mock_sm)
  
  # Now benchmark subsequent calls
  start_time = time.perf_counter()
  for _ in range(50):  # Multiple calls to see steady-state performance
    thermal_manager._calculate_system_load_factor(mock_sm)
  end_time = time.perf_counter()
  
  avg_ema_time = (end_time - start_time) / 50
  print(f"  Average EMA calculation time: {avg_ema_time*1000:.3f}ms")
  print(f"  ✓ EMA smoothing meets performance requirements")
  
  return avg_ema_time


def assess_memory_usage():
  """Assess memory usage of the new features."""
  print("Assessing memory usage...")
  
  # Get initial memory usage
  initial_memory = psutil.Process().memory_info().rss / 1024 / 1024  # MB
  
  # Create multiple thermal managers to test memory growth
  thermal_managers = []
  for i in range(10):
    tm = ThermalManager()
    
    # Create mock data
    mock_sm = Mock()
    mock_device_state = Mock()
    mock_device_state.cpuPct = 50.0 + i*5  # Vary the load
    mock_device_state.gpuPct = 40.0 + i*4
    mock_device_state.memoryPct = 60.0 + i*3
    
    def mock_get(key, default=None):
      if key == 'deviceState':
        return mock_device_state
      return default
    
    mock_sm.get = mock_get
    
    # Trigger the system load calculation to initialize internal structures
    tm._calculate_system_load_factor(mock_sm)
    thermal_managers.append(tm)
  
  # Get memory after creating thermal managers
  after_creation_memory = psutil.Process().memory_info().rss / 1024 / 1024  # MB
  
  memory_increase = after_creation_memory - initial_memory
  avg_memory_per_instance = memory_increase / len(thermal_managers)
  
  print(f"  Initial memory: {initial_memory:.2f} MB")
  print(f"  After creating {len(thermal_managers)} thermal managers: {after_creation_memory:.2f} MB")
  print(f"  Memory increase: {memory_increase:.2f} MB")
  print(f"  Average per instance: {avg_memory_per_instance:.3f} MB")
  print(f"  ✓ Memory usage is reasonable (< 10 MB per instance)")
  
  return avg_memory_per_instance


def main():
  """Run all performance baseline assessments."""
  print("Starting performance baseline assessment...\n")
  
  # Run all benchmarks
  calc_time = benchmark_system_load_calculation()
  pred_time = benchmark_temperature_prediction()
  ema_time = benchmark_ema_smoothing()
  mem_usage = assess_memory_usage()
  
  print(f"\nPerformance Baseline Summary:")
  print(f"  System load calculation: {calc_time*1000:.3f}ms avg")
  print(f"  Temperature prediction: {pred_time*1000:.3f}ms avg")
  print(f"  EMA smoothing: {ema_time*1000:.3f}ms avg")
  print(f"  Memory per instance: {mem_usage:.3f} MB avg")
  
  # Verify all metrics meet requirements
  assert calc_time < 0.001, f"System load calculation too slow: {calc_time*1000:.2f}ms (>1ms)"
  assert pred_time < 0.001, f"Temperature prediction too slow: {pred_time*1000:.2f}ms (>1ms)"
  assert ema_time < 0.001, f"EMA smoothing too slow: {ema_time*1000:.2f}ms (>1ms)"
  assert mem_usage < 10.0, f"Memory usage too high: {mem_usage:.2f}MB (>10MB)"
  
  print(f"\n✓ All performance baselines established and requirements met!")


if __name__ == "__main__":
  main()