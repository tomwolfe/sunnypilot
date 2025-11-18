"""
Hardware monitoring module for tracking CPU, RAM, and power usage
on the comma three hardware platform.
"""
import time
import psutil
import threading
import os
from typing import Dict, Optional
from openpilot.selfdrive.common.metrics import Metrics, record_metric

class HardwareMonitor:
    """Monitors hardware resource usage on the comma three platform."""
    
    def __init__(self, update_interval: float = 1.0):
        self.update_interval = update_interval
        self.monitoring = False
        self.monitor_thread = None
        self.last_update_time = time.time()
        
        # Hardware limits for comma three (2GB RAM, 4-core ARM CPU)
        self.ram_limit_mb = 2048  # 2GB in MB
        self.cpu_limit_percent = 100.0  # 100% per core
        
    def start_monitoring(self):
        """Start the hardware monitoring thread."""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
    
    def stop_monitoring(self):
        """Stop the hardware monitoring thread."""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()
    
    def _monitor_loop(self):
        """Main monitoring loop that runs in a separate thread."""
        while self.monitoring:
            self._update_metrics()
            time.sleep(self.update_interval)
    
    def _update_metrics(self):
        """Update hardware metrics and record them."""
        try:
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=None)
            cpu_per_core = psutil.cpu_percent(interval=None, percpu=True)
            
            # Record overall CPU usage
            record_metric(Metrics.CPU_USAGE_PERCENT, cpu_percent, {
                "source": "system",
                "per_core": cpu_per_core
            })
            
            # RAM usage
            memory = psutil.virtual_memory()
            ram_used_mb = memory.used / (1024 * 1024)  # Convert to MB
            ram_percent = memory.percent
            
            # Record RAM metrics
            record_metric(Metrics.RAM_USAGE_MB, ram_used_mb, {
                "source": "system",
                "total_mb": self.ram_limit_mb
            })
            
            record_metric(Metrics.RAM_USAGE_PERCENT, ram_percent, {
                "source": "system"
            })
            
            # Additional system metrics
            load_avg = psutil.getloadavg()
            record_metric("hardware.system.load_avg_1min", load_avg[0], {"source": "system"})
            record_metric("hardware.system.load_avg_5min", load_avg[1], {"source": "system"})
            record_metric("hardware.system.load_avg_15min", load_avg[2], {"source": "system"})
            
            # Process-specific metrics for current process
            current_process = psutil.Process()
            proc_memory_mb = current_process.memory_info().rss / (1024 * 1024)
            record_metric("hardware.process.ram_usage_mb", proc_memory_mb, {
                "pid": current_process.pid,
                "source": "process"
            })
            
            # Record power-related metrics (if available on the platform)
            # For comma three, we can approximate power based on CPU usage and thermal data
            # This is a simplified calculation - real power monitoring would require access to hardware registers
            estimated_power = self._estimate_power(cpu_percent, ram_used_mb)
            record_metric(Metrics.POWER_DRAW_WATTS, estimated_power, {
                "calculation_method": "estimated_from_cpu_ram",
                "cpu_percent": cpu_percent,
                "ram_mb": ram_used_mb
            })
            
            # Check for violations of hardware constraints
            if cpu_percent > 35.0:  # Target <35% CPU usage
                record_metric(Metrics.CPU_USAGE_PERCENT, cpu_percent, {
                    "source": "violation",
                    "violation": "CPU usage exceeded 35%",
                    "threshold": 35.0
                })
            
            if ram_used_mb > 1433.6:  # Target <1.4GB (1433.6 MB)
                record_metric(Metrics.RAM_USAGE_MB, ram_used_mb, {
                    "source": "violation",
                    "violation": "RAM usage exceeded 1.4GB",
                    "threshold_mb": 1433.6
                })
                
        except Exception as e:
            # Log the error but don't crash the monitoring
            print(f"Hardware monitoring error: {e}")
    
    def _estimate_power(self, cpu_percent: float, ram_mb: float) -> float:
        """
        Estimate power consumption based on CPU and RAM usage.
        This is a simplified model for the comma three platform.
        """
        # Base power consumption (idle state)
        base_power = 2.0  # watts
        
        # CPU power component (scales with usage)
        cpu_power_factor = 0.08  # additional watts per percent CPU
        cpu_component = (cpu_percent / 100.0) * 8.0  # max additional ~6.4W at 100% CPU
        
        # RAM power component (roughly proportional to usage)
        ram_power_factor = 0.0001  # additional watts per MB
        ram_component = (ram_mb / self.ram_limit_mb) * 0.5  # max additional ~0.5W
        
        estimated_power = base_power + cpu_component + ram_component
        
        # Cap at 8W target to represent the power budget constraint
        return min(estimated_power, 8.0)

# Global hardware monitor instance
hardware_monitor = HardwareMonitor(update_interval=1.0)

def start_hardware_monitoring():
    """Start hardware resource monitoring."""
    hardware_monitor.start_monitoring()

def stop_hardware_monitoring():
    """Stop hardware resource monitoring."""
    hardware_monitor.stop_monitoring()

def get_hardware_metrics() -> Dict[str, float]:
    """Get current hardware metrics."""
    try:
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory = psutil.virtual_memory()
        ram_used_mb = memory.used / (1024 * 1024)
        return {
            "cpu_percent": cpu_percent,
            "ram_mb": ram_used_mb,
            "ram_percent": memory.percent
        }
    except Exception as e:
        print(f"Error getting hardware metrics: {e}")
        return {}