"""
Performance monitoring utilities for sunnypilot
Provides runtime performance tracking for critical systems
"""
import time
import threading
import json # Import json for loading performance baselines
from collections import deque
from typing import Dict, Optional
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