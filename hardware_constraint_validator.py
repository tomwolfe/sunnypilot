#!/usr/bin/env python3
"""
Hardware Constraint Validator for Sunnypilot
Validates that all systems stay within Comma 3x hardware constraints
"""
import psutil
import os
import time
import numpy as np
from typing import Dict, Tuple
import threading
from collections import deque

from openpilot.common.swaglog import cloudlog


class HardwareConstraintValidator:
    """Validates system performance stays within Comma 3x constraints"""
    
    def __init__(self):
        self.constraints = {
            'max_ram_usage': 1400 * 1024 * 1024,  # 1.4GB in bytes
            'max_cpu_usage': 5.0,  # 5% average during normal operation
            'max_peak_cpu': 10.0,   # 10% peak during high-load
            'max_latency': 0.08,    # 80ms end-to-end latency target
        }
        
        self.monitoring = False
        self.metrics_history = {
            'cpu_usage': deque(maxlen=1000),  # Last 1000 measurements
            'memory_usage': deque(maxlen=1000),
            'latency': deque(maxlen=1000)
        }
        self.start_time = time.time()

    def get_system_metrics(self) -> Dict[str, float]:
        """Get current system metrics"""
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory_info = psutil.virtual_memory()
        memory_usage_bytes = memory_info.used
        memory_percent = memory_info.percent
        
        return {
            'cpu_percent': cpu_percent,
            'memory_bytes': memory_usage_bytes,
            'memory_percent': memory_percent,
            'uptime_seconds': time.time() - self.start_time,
            'timestamp': time.time()
        }

    def validate_constraints(self, metrics: Dict[str, float]) -> Tuple[bool, Dict[str, str]]:
        """Validate metrics against hardware constraints"""
        violations = {}
        
        # Check RAM usage
        if metrics['memory_bytes'] > self.constraints['max_ram_usage']:
            violations['ram'] = f"RAM usage {metrics['memory_bytes']/1024/1024:.1f}MB exceeds limit {self.constraints['max_ram_usage']/1024/1024:.1f}MB"
        
        # Check CPU usage - needs to account for average over time
        self.metrics_history['cpu_usage'].append(metrics['cpu_percent'])
        if len(self.metrics_history['cpu_usage']) > 10:  # Only calculate average after 10 samples
            avg_cpu = np.mean(list(self.metrics_history['cpu_usage']))
        else:
            avg_cpu = metrics['cpu_percent']
        
        if avg_cpu > self.constraints['max_cpu_usage']:
            violations['cpu_avg'] = f"Average CPU usage {avg_cpu:.1f}% exceeds limit {self.constraints['max_cpu_usage']:.1f}%"
            
        if metrics['cpu_percent'] > self.constraints['max_peak_cpu']:
            violations['cpu_peak'] = f"Peak CPU usage {metrics['cpu_percent']:.1f}% exceeds limit {self.constraints['max_peak_cpu']:.1f}%"
        
        return len(violations) == 0, violations

    def start_monitoring(self):
        """Start monitoring in background thread"""
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        cloudlog.info("Hardware constraint monitoring started")

    def stop_monitoring(self):
        """Stop monitoring"""
        self.monitoring = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=1.0)
        cloudlog.info("Hardware constraint monitoring stopped")

    def _monitor_loop(self):
        """Background monitoring loop"""
        while self.monitoring:
            try:
                metrics = self.get_system_metrics()
                is_valid, violations = self.validate_constraints(metrics)
                
                if not is_valid:
                    for constraint, msg in violations.items():
                        cloudlog.error(f"Hardware constraint violation - {msg}")
                else:
                    # Log performance metrics periodically
                    if int(metrics['uptime_seconds']) % 30 == 0:  # Every 30 seconds
                        if len(self.metrics_history['cpu_usage']) > 10:
                            avg_cpu = np.mean(list(self.metrics_history['cpu_usage']))
                            avg_memory = np.mean(list(self.metrics_history['memory_usage']))
                            cloudlog.debug(f"Performance metrics - CPU: {avg_cpu:.1f}%, RAM: {avg_memory/1024/1024:.1f}MB")
                
                time.sleep(0.5)  # Monitor every 500ms
            except Exception as e:
                cloudlog.error(f"Error in hardware monitoring: {e}")
                time.sleep(1.0)

    def get_performance_report(self) -> Dict[str, float]:
        """Generate performance report"""
        if len(self.metrics_history['cpu_usage']) == 0:
            return {"error": "No metrics collected yet"}
            
        report = {
            'avg_cpu_percent': float(np.mean(list(self.metrics_history['cpu_usage']))),
            'max_cpu_percent': float(np.max(list(self.metrics_history['cpu_usage']))),
            'avg_memory_mb': float(np.mean(list(self.metrics_history['memory_usage']))) / (1024*1024),
            'max_memory_mb': float(np.max(list(self.metrics_history['memory_usage']))) / (1024*1024),
            'uptime_hours': (time.time() - self.start_time) / 3600,
            'samples_collected': len(self.metrics_history['cpu_usage'])
        }
        return report


def validate_hardware_constraints():
    """Run hardware constraint validation"""
    validator = HardwareConstraintValidator()
    
    # Run continuous monitoring
    validator.start_monitoring()
    
    try:
        # Let it run for a while to collect metrics
        time.sleep(10)  # Monitor for 10 seconds
        
        # Generate report
        report = validator.get_performance_report()
        print("Hardware Constraint Validation Report:")
        print(f"Average CPU Usage: {report.get('avg_cpu_percent', 'N/A'):.2f}%")
        print(f"Max CPU Usage: {report.get('max_cpu_percent', 'N/A'):.2f}%")
        print(f"Average Memory Usage: {report.get('avg_memory_mb', 'N/A'):.2f} MB")
        print(f"Max Memory Usage: {report.get('max_memory_mb', 'N/A'):.2f} MB")
        print(f"Monitoring Duration: {report.get('uptime_hours', 0):.2f} hours")
        print(f"Samples Collected: {report.get('samples_collected', 0)}")
        
        # Check if within constraints
        cpu_valid = report.get('avg_cpu_percent', 100) <= 5  # Target: <5% average
        mem_valid = report.get('avg_memory_mb', 2000) <= 1400  # Target: <1.4GB
        
        print(f"CPU Constraint Compliance: {'✅ PASS' if cpu_valid else '❌ FAIL'}")
        print(f"Memory Constraint Compliance: {'✅ PASS' if mem_valid else '❌ FAIL'}")
        
        return cpu_valid and mem_valid
        
    finally:
        validator.stop_monitoring()


if __name__ == "__main__":
    success = validate_hardware_constraints()
    exit(0 if success else 1)