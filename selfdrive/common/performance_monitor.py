"""
Performance Monitoring for Sunnypilot
Utilities for monitoring and optimizing system performance
"""
import time
import psutil
from typing import Dict, Any, Callable
from functools import wraps
from openpilot.common.swaglog import cloudlog


class PerformanceMonitor:
  """System performance monitoring with detailed metrics"""
  
  def __init__(self):
    self.function_call_times = {}
    self.cpu_threshold = 80.0
    self.memory_threshold = 80.0
    
  def monitor_performance(self, func_name: str = None):
    """Decorator to monitor function performance"""
    def decorator(func):
      name = func_name or func.__name__
      
      @wraps(func)
      def wrapper(*args, **kwargs):
        start_time = time.time()
        start_cpu = psutil.cpu_percent()
        start_memory = psutil.virtual_memory().percent
        
        result = func(*args, **kwargs)
        
        end_time = time.time()
        duration = end_time - start_time
        
        # Store performance data
        if name not in self.function_call_times:
          self.function_call_times[name] = []
          
        self.function_call_times[name].append({
          'duration': duration,
          'cpu_percent': start_cpu,
          'memory_percent': start_memory,
          'timestamp': start_time
        })
        
        # Log warnings for performance issues
        if duration > 0.1:  # More than 100ms
          cloudlog.warning(f"Function {name} took {duration:.3f}s to execute")
          
        return result
      return wrapper
    return decorator
  
  def get_performance_report(self) -> Dict[str, Any]:
    """Get overall performance report"""
    report = {
      'timestamp': time.time(),
      'cpu_percent': psutil.cpu_percent(),
      'memory_percent': psutil.virtual_memory().percent,
      'disk_percent': psutil.disk_usage('/').percent,
      'function_performance': {}
    }
    
    # Calculate average execution times for each function
    for func_name, times in self.function_call_times.items():
      recent_calls = times[-10:]  # Last 10 calls
      avg_duration = sum(call['duration'] for call in recent_calls) / len(recent_calls)
      
      report['function_performance'][func_name] = {
        'avg_duration': avg_duration,
        'call_count': len(recent_calls),
        'max_duration': max(call['duration'] for call in recent_calls),
        'min_duration': min(call['duration'] for call in recent_calls)
      }
    
    return report


# Global performance monitor instance
perf_monitor = PerformanceMonitor()