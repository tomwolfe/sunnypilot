"""
Performance monitoring utilities for sunnypilot
Provides runtime performance tracking for critical systems
"""
import time
import threading
import json # Import json for loading performance baselines
from collections import deque
from typing import Dict, Optional, Tuple
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params, UnknownKeyName # Import Params and UnknownKeyName


class PerformanceTimer:
  """Simple timer for measuring execution time of code blocks"""
  def __init__(self, name: str, enabled: bool = True):
    self.name = name
    self.enabled = enabled
    self.start_time = 0.0
    self.elapsed = 0.0

  def __enter__(self):
    if self.enabled:
      self.start_time = time.perf_counter()
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    if self.enabled:
      self.elapsed = time.perf_counter() - self.start_time
      cloudlog.debug(f"{self.name} took {self.elapsed*1000:.2f}ms")

  def get_time_ms(self) -> float:
    return self.elapsed * 1000


class PerformanceMonitor:
  """Monitors performance of critical systems"""
  def __init__(self):
    self.timers: Dict[str, deque] = {}
    self.timer_lock = threading.Lock()
    
    self.params = Params() # Initialize Params

    try:
        max_samples_param = self.params.get("PerformanceMonitorMaxSamples")
        self.max_samples = int(float(max_samples_param)) if max_samples_param else 50 # Default to 50 samples
    except (UnknownKeyName, ValueError):
        self.max_samples = 50 # Default value if parameter not found or invalid

    # Performance baselines (configurable via Params)
    try:
        baselines_str = self.params.get("PerformanceBaselines")
        # Example: '{"lateral_accuracy": 0.1, "longitudinal_accuracy": 0.2, "ride_comfort": 0.9}'
        self.performance_baselines = json.loads(baselines_str) if baselines_str else {
            'lateral_accuracy': 0.1,
            'longitudinal_accuracy': 0.2,
            'ride_comfort': 0.9
        }
    except (UnknownKeyName, ValueError, json.JSONDecodeError):
        self.performance_baselines = {
            'lateral_accuracy': 0.1,
            'longitudinal_accuracy': 0.2,
            'ride_comfort': 0.9
        }
    
    # Window size for performance health check
    try:
        health_window_param = self.params.get("PerformanceHealthWindow")
        self.performance_health_window = int(float(health_window_param)) if health_window_param else 10 # Default to 10 samples
    except (UnknownKeyName, ValueError):
        self.performance_health_window = 10

    self.performance_unhealthy_counter = 0
    self.performance_unhealthy = False

  def add_timing(self, name: str, elapsed_ms: float):
    """Add a timing sample for a named operation"""
    with self.timer_lock:
      if name not in self.timers:
        self.timers[name] = deque(maxlen=self.max_samples)
      self.timers[name].append(elapsed_ms)

  def get_average_time(self, name: str) -> Optional[float]:
    """Get average execution time for named operation"""
    with self.timer_lock:
      if name not in self.timers or len(self.timers[name]) == 0:
        return None
      return sum(self.timers[name]) / len(self.timers[name])

  def get_max_time(self, name: str) -> Optional[float]:
    """Get maximum execution time for named operation"""
    with self.timer_lock:
      if name not in self.timers or len(self.timers[name]) == 0:
        return None
      return max(self.timers[name])

  def get_metrics(self) -> Dict[str, Dict[str, float]]:
    """Get all performance metrics"""
    with self.timer_lock:
      metrics = {}
      for name, times in self.timers.items():
        if len(times) > 0:
          metrics[name] = {
            'avg': sum(times) / len(times),
            'max': max(times),
            'min': min(times),
            'count': len(times)
          }
      return metrics

  def evaluate_performance(self, desired_state: Dict, actual_state: Dict, model_output: Dict, control_output: Dict) -> Dict:
    """
    Evaluates current system performance against baselines and computes metrics.
    This method should be called periodically by the main control loop.
    """
    performance_metrics = {}

    # Lateral accuracy: Deviation from desired path
    # The actual lateral deviation is available as 'lateral_deviation' in actual_state.
    # We should use this as the primary metric for lateral accuracy.
    # The model output contains path information that can also be used.
    if 'lateral_deviation' in actual_state:
        performance_metrics['lateral_accuracy'] = abs(actual_state['lateral_deviation'])
    elif 'lateral' in actual_state: # Fallback for backward compatibility, though not preferred
        performance_metrics['lateral_accuracy'] = actual_state['lateral']
    else:
        performance_metrics['lateral_accuracy'] = float('inf') # Indicate no valid data

    # Longitudinal accuracy: Difference between desired speed and actual speed
    if 'longitudinal' in desired_state and 'longitudinal' in actual_state:
        performance_metrics['longitudinal_accuracy'] = abs(desired_state['longitudinal'] - actual_state['longitudinal'])
    else:
        performance_metrics['longitudinal_accuracy'] = float('inf')

    # Path deviation: Difference between desired and actual curvature
    if 'path_deviation' in desired_state: # This is already the difference
        performance_metrics['path_deviation'] = abs(desired_state['path_deviation'])
    else:
        performance_metrics['path_deviation'] = float('inf')

    # Ride comfort: Based on jerk or acceleration changes
    if 'jerk' in control_output:
        performance_metrics['ride_comfort'] = abs(control_output['jerk']) # Lower is better
    elif 'lateral_accel' in actual_state:
        performance_metrics['ride_comfort'] = abs(actual_state['lateral_accel'])
    else:
        performance_metrics['ride_comfort'] = float('inf')

    # Check for performance degradation against baselines
    self._check_performance_degradation(performance_metrics)

    return performance_metrics

  def _check_performance_degradation(self, current_metrics: Dict):
    """
    Internal method to check if current performance metrics indicate degradation
    based on predefined baselines.
    """
    is_degraded = False
    degradation_details = {}

    for metric, value in current_metrics.items():
        if metric in self.performance_baselines:
            baseline = self.performance_baselines[metric]
            # For accuracy metrics, higher value means worse performance, so check if current > baseline
            # For ride_comfort (jerk), higher value means worse, so check if current > baseline
            if value > baseline:
                is_degraded = True
                degradation_details[metric] = f"Current {value:.2f} > Baseline {baseline:.2f}"
    
    # Update performance unhealthy status and counter for hysteresis
    if is_degraded:
        self.performance_unhealthy_counter = min(self.performance_unhealthy_counter + 1, self.performance_health_window + 1)
        if self.performance_unhealthy_counter >= self.performance_health_window:
            self.performance_unhealthy = True
            cloudlog.warning(f"Performance degraded: {degradation_details}")
    else:
        self.performance_unhealthy_counter = max(self.performance_unhealthy_counter - 1, 0)
        if self.performance_unhealthy_counter == 0:
            self.performance_unhealthy = False

  def should_adapt_parameters(self) -> Tuple[bool, Optional[Dict]]:
    """
    Determines if tuning parameters should be adapted based on performance degradation.
    This is a placeholder for adaptive control logic.
    Returns: (bool, Optional[Dict]) - (should_adapt, suggested_params_for_adaptation)
    """
    if self.performance_unhealthy:
        # Example: if lateral accuracy is consistently poor, suggest adjusting lateral tuning
        if 'lateral_accuracy' in self.performance_baselines and \
           self.performance_unhealthy_counter >= self.performance_health_window:
            
            # This is a very basic example. Real adaptation would use more sophisticated logic.
            # For instance, increasing a P gain if there's persistent understeer.
            suggested_params = {
                'lateral_kp_factor': 1.05, # Example: increase Kp by 5%
                'lateral_ki_factor': 1.02, # Example: increase Ki by 2%
            }
            return True, suggested_params
    return False, None

# Global performance monitor instance
perf_monitor = PerformanceMonitor()


def measure_time(name: str):
  """Decorator to measure execution time of a function"""
  def decorator(func):
    def wrapper(*args, **kwargs):
      start = time.perf_counter()
      result = func(*args, **kwargs)
      elapsed = (time.perf_counter() - start) * 1000  # Convert to milliseconds
      perf_monitor.add_timing(name, elapsed)
      return result
    return wrapper
  return decorator


# Context manager for performance measurement
class PerfTrack:
  """Context manager for performance tracking"""
  def __init__(self, name: str):
    self.name = name
    self.start_time = 0.0

  def __enter__(self):
    self.start_time = time.perf_counter()
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    elapsed = (time.perf_counter() - self.start_time) * 1000  # Convert to milliseconds
    perf_monitor.add_timing(self.name, elapsed)

  def get_time_ms(self) -> float:
    return (time.perf_counter() - self.start_time) * 1000