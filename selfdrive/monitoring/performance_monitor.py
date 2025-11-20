"""
Performance Monitoring Utility for Autonomous Driving System
Tracks actual performance metrics and overhead of monitoring system
"""
import time
import threading
from typing import Dict, Any, Callable
from collections import deque
import numpy as np

from openpilot.common.swaglog import cloudlog


class PerformanceTracker:
    """Tracks performance of various system components"""
    
    def __init__(self, max_samples: int = 100):
        self.max_samples = max_samples
        self.execution_times = deque(maxlen=max_samples)
        self.start_time = None
        self.component_name = ""
        
    def start_timing(self, component_name: str = "unknown"):
        """Start timing for a specific component"""
        self.component_name = component_name
        self.start_time = time.perf_counter()
        
    def end_timing(self) -> float:
        """End timing and return execution time in milliseconds"""
        if self.start_time is None:
            return 0.0
            
        execution_time = (time.perf_counter() - self.start_time) * 1000  # Convert to ms
        self.execution_times.append(execution_time)
        self.start_time = None
        
        return execution_time
        
    def get_stats(self) -> Dict[str, float]:
        """Get performance statistics"""
        if not self.execution_times:
            return {
                'avg_time_ms': 0.0,
                'min_time_ms': 0.0,
                'max_time_ms': 0.0,
                'std_time_ms': 0.0,
                'samples': 0
            }
            
        times_array = np.array(self.execution_times)
        return {
            'avg_time_ms': float(np.mean(times_array)),
            'min_time_ms': float(np.min(times_array)),
            'max_time_ms': float(np.max(times_array)),
            'std_time_ms': float(np.std(times_array)),
            'samples': len(self.execution_times)
        }


class SystemPerformanceMonitor:
    """Main performance monitoring system"""
    
    def __init__(self):
        self.trackers: Dict[str, PerformanceTracker] = {}
        self.enabled = True
        self.monitoring_thread = None
        self.stop_event = threading.Event()
        
    def get_tracker(self, component_name: str) -> PerformanceTracker:
        """Get or create a performance tracker for a component"""
        if component_name not in self.trackers:
            self.trackers[component_name] = PerformanceTracker()
        return self.trackers[component_name]
    
    def time_function(self, component_name: str, func: Callable, *args, **kwargs) -> Any:
        """Time the execution of a function"""
        if not self.enabled:
            return func(*args, **kwargs)
            
        tracker = self.get_tracker(component_name)
        tracker.start_timing(component_name)
        
        try:
            result = func(*args, **kwargs)
            execution_time = tracker.end_timing()
            
            # Log if execution time is excessive
            if execution_time > 20.0:  # More than 20ms
                cloudlog.warning(f"{component_name} took {execution_time:.2f}ms, which exceeds performance target")
                
            return result
        except Exception as e:
            tracker.end_timing()  # End timing even if function fails
            raise e
    
    def get_all_stats(self) -> Dict[str, Dict[str, float]]:
        """Get performance statistics for all components"""
        stats = {}
        for name, tracker in self.trackers.items():
            stats[name] = tracker.get_stats()
        return stats
    
    def print_performance_report(self):
        """Print a comprehensive performance report"""
        stats = self.get_all_stats()
        
        print("=" * 80)
        print("SYSTEM PERFORMANCE REPORT")
        print("=" * 80)
        
        for component, stat in stats.items():
            avg_time = stat['avg_time_ms']
            max_time = stat['max_time_ms']
            samples = stat['samples']
            
            performance_level = "EXCELLENT" if avg_time < 5.0 else \
                              "GOOD" if avg_time < 10.0 else \
                              "FAIR" if avg_time < 20.0 else \
                              "POOR"
            
            print(f"{component:25} | AVG: {avg_time:6.2f}ms | MAX: {max_time:6.2f}ms | "
                  f"SAMPLES: {samples:3d} | {performance_level}")
        
        print("=" * 80)
    
    def start_monitoring_loop(self, interval: float = 10.0):
        """Start a background monitoring loop"""
        def monitoring_worker():
            while not self.stop_event.wait(interval):
                self.print_performance_report()
                
        self.monitoring_thread = threading.Thread(target=monitoring_worker, daemon=True)
        self.monitoring_thread.start()
        
    def stop_monitoring(self):
        """Stop the monitoring system"""
        self.stop_event.set()
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)


# Global performance monitor instance
_performance_monitor = SystemPerformanceMonitor()


def get_performance_monitor() -> SystemPerformanceMonitor:
    """Get the global performance monitor instance"""
    return _performance_monitor


def time_critical_function(component_name: str, func: Callable, *args, **kwargs) -> Any:
    """Convenience function to time a critical function"""
    return _performance_monitor.time_function(component_name, func, *args, **kwargs)


if __name__ == "__main__":
    # Example usage
    monitor = get_performance_monitor()
    
    # Example function to time
    def example_function():
        time.sleep(0.01)  # Simulate 10ms of work
        return "result"
    
    # Time the function
    result = time_critical_function("example_func", example_function)
    
    # Print report
    monitor.print_performance_report()