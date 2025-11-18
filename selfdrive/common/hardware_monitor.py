"""
Hardware monitoring module for comma three platform.
Monitors CPU, RAM, and power usage to ensure compliance with hardware constraints.
"""
import time
import threading
import psutil
import subprocess
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
from selfdrive.common.metrics import Metrics, record_metric

@dataclass
class HardwareStatus:
    """Represents current hardware status."""
    cpu_percent: float
    cpu_per_core: list
    ram_used_mb: float
    ram_percent: float
    power_estimate_w: float
    timestamp: float

class HardwareMonitor:
    """
    Real-time hardware monitor for comma three platform.
    Monitors CPU, RAM, and power usage with ARM-specific optimizations.
    """
    
    def __init__(self, update_interval: float = 1.0):
        self.update_interval = update_interval
        self.current_status: Optional[HardwareStatus] = None
        self.is_monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        
        # Hardware limits for comma three
        self.cpu_limit = 35.0  # Target: <35% average CPU usage
        self.ram_limit = 1433.6  # Target: <1.4GB (1433.6MB) RAM usage
        self.power_limit = 8.0  # Target: <8W power draw
        
        # ARM-specific optimizations
        self.is_arm_processor = self._check_arm_processor()
        self.core_count = psutil.cpu_count()
        
    def _check_arm_processor(self) -> bool:
        """Check if running on ARM processor."""
        try:
            result = subprocess.run(['uname', '-m'], capture_output=True, text=True)
            return 'arm' in result.stdout.lower() or 'aarch' in result.stdout.lower()
        except:
            return False
            
    def get_hardware_status(self) -> HardwareStatus:
        """Get current hardware status."""
        # Get CPU metrics
        cpu_percent = psutil.cpu_percent(interval=0.1)  # Brief interval to avoid blocking
        cpu_per_core = psutil.cpu_percent(percpu=True)
        
        # Get RAM metrics
        memory = psutil.virtual_memory()
        ram_used_mb = memory.used / (1024**2)  # Convert to MB
        ram_percent = memory.percent
        
        # Estimate power consumption based on ARM characteristics
        power_w = self._estimate_power(cpu_percent, ram_used_mb)
        
        status = HardwareStatus(
            cpu_percent=cpu_percent,
            cpu_per_core=cpu_per_core,
            ram_used_mb=ram_used_mb,
            ram_percent=ram_percent,
            power_estimate_w=power_w,
            timestamp=time.time()
        )
        
        # Record metrics for tracking
        record_metric(Metrics.CPU_USAGE_PERCENT, cpu_percent, {
            "hardware_component": "cpu",
            "per_core": cpu_per_core,
            "arm_processor": self.is_arm_processor
        })
        
        record_metric(Metrics.RAM_USAGE_MB, ram_used_mb, {
            "hardware_component": "ram",
            "percent_used": ram_percent,
            "total_available_mb": memory.total / (1024**2)
        })
        
        record_metric(Metrics.POWER_DRAW_WATTS, power_w, {
            "estimated_power_w": power_w,
            "based_on_cpu_percent": cpu_percent,
            "based_on_ram_mb": ram_used_mb,
            "arm_processor": self.is_arm_processor
        })
        
        return status
        
    def _estimate_power(self, cpu_percent: float, ram_used_mb: float) -> float:
        """
        Estimate power consumption based on ARM processor characteristics.
        Uses a simplified model based on real ARM power consumption data.
        """
        # Base power consumption for ARM SoC on comma three
        base_power = 0.8  # Base power in watts
        
        # CPU power component - follows cubic relationship for ARM processors
        # At 100% CPU, ARM processors consume significantly more power
        cpu_power = base_power * ((cpu_percent / 100.0) ** 2.5) * 4.0
        
        # RAM power component - roughly linear with usage
        # 2GB RAM on comma three draws additional power based on usage
        total_ram_mb = psutil.virtual_memory().total / (1024**2)
        ram_power = 0.5 * (ram_used_mb / total_ram_mb)
        
        # Total estimated power
        estimated_power = base_power + cpu_power + ram_power
        
        # Cap at reasonable upper limit for comma three
        return min(estimated_power, 10.0)  # Cap at 10W for safety
        
    def start_monitoring(self):
        """Start continuous hardware monitoring."""
        if self.is_monitoring:
            return
            
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
    def stop_monitoring(self):
        """Stop continuous hardware monitoring."""
        self.is_monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()
            
    def _monitoring_loop(self):
        """Internal monitoring loop running in separate thread."""
        while self.is_monitoring:
            try:
                self.current_status = self.get_hardware_status()
                
                # Check compliance with hardware limits
                cpu_compliant = self.current_status.cpu_percent < self.cpu_limit
                ram_compliant = self.current_status.ram_used_mb < self.ram_limit
                power_compliant = self.current_status.power_estimate_w < self.power_limit
                
                # Record compliance status
                record_metric("hardware.cpu_compliant", 1.0 if cpu_compliant else 0.0, {
                    "current_usage": self.current_status.cpu_percent,
                    "limit": self.cpu_limit
                })
                
                record_metric("hardware.ram_compliant", 1.0 if ram_compliant else 0.0, {
                    "current_usage_mb": self.current_status.ram_used_mb,
                    "limit_mb": self.ram_limit
                })
                
                record_metric("hardware.power_compliant", 1.0 if power_compliant else 0.0, {
                    "current_usage_w": self.current_status.power_estimate_w,
                    "limit_w": self.power_limit
                })
                
                # Warn if exceeding limits
                violations = []
                if not cpu_compliant:
                    violations.append(f"CPU {self.current_status.cpu_percent:.1f}% > {self.cpu_limit}%")
                if not ram_compliant:
                    violations.append(f"RAM {self.current_status.ram_used_mb:.1f}MB > {self.ram_limit}MB")
                if not power_compliant:
                    violations.append(f"Power {self.current_status.power_estimate_w:.2f}W > {self.power_limit}W")
                    
                if violations:
                    print(f"Hardware monitoring: Violations detected - {', '.join(violations)}")
                
                time.sleep(self.update_interval)
                
            except Exception as e:
                print(f"Hardware monitor error: {e}")
                time.sleep(self.update_interval)
                
    def is_within_limits(self) -> bool:
        """Check if all hardware metrics are within limits."""
        if not self.current_status:
            return False
            
        return (
            self.current_status.cpu_percent < self.cpu_limit and
            self.current_status.ram_used_mb < self.ram_limit and
            self.current_status.power_estimate_w < self.power_limit
        )
        
    def get_limit_compliance_report(self) -> Dict[str, Any]:
        """Get detailed compliance report."""
        if not self.current_status:
            return {"error": "No hardware status available"}
            
        return {
            "cpu_within_limit": self.current_status.cpu_percent < self.cpu_limit,
            "current_cpu": self.current_status.cpu_percent,
            "cpu_limit": self.cpu_limit,
            
            "ram_within_limit": self.current_status.ram_used_mb < self.ram_limit,
            "current_ram_mb": self.current_status.ram_used_mb,
            "ram_limit_mb": self.ram_limit,
            
            "power_within_limit": self.current_status.power_estimate_w < self.power_limit,
            "current_power_w": self.current_status.power_estimate_w,
            "power_limit_w": self.power_limit,
            
            "timestamp": self.current_status.timestamp
        }

# Global hardware monitor instance
hardware_monitor = HardwareMonitor(update_interval=1.0)

def get_hardware_monitor() -> HardwareMonitor:
    """Get the global hardware monitor instance."""
    return hardware_monitor

def start_hardware_monitoring():
    """Start the global hardware monitoring."""
    global hardware_monitor
    hardware_monitor.start_monitoring()

def stop_hardware_monitoring():
    """Stop the global hardware monitoring."""
    global hardware_monitor
    hardware_monitor.stop_monitoring()

def get_current_hardware_status() -> Optional[HardwareStatus]:
    """Get current hardware status from global monitor."""
    global hardware_monitor
    return hardware_monitor.current_status

def is_hardware_within_limits() -> bool:
    """Check if global hardware monitor reports compliance."""
    global hardware_monitor
    return hardware_monitor.is_within_limits()