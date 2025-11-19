#!/usr/bin/env python3
"""
System Health Monitoring for Sunnypilot
Provides comprehensive system health monitoring for the autonomous driving system.
"""

import time
import psutil
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import threading
import logging
import json
from pathlib import Path


@dataclass
class SystemHealthReport:
    """Dataclass for system health report."""
    timestamp: float
    cpu_usage: float
    memory_usage: float
    disk_usage: float
    thermal_sensors: Dict[str, float]
    system_status: str
    warnings: List[str]
    critical_issues: List[str]
    network_stats: Dict[str, Any]  # Additional network statistics
    battery_status: Optional[Dict[str, Any]]  # Battery info if available
    uptime: float  # System uptime in seconds
    process_stats: Dict[str, Any]  # Process-related stats


class SystemHealthMonitor:
    """Comprehensive system health monitoring for sunnypilot."""

    def __init__(self,
                 cpu_thresholds: Dict[str, float] = None,
                 memory_thresholds: Dict[str, float] = None,
                 disk_thresholds: Dict[str, float] = None,
                 log_file: Optional[str] = None):
        """Initialize the health monitor with configurable thresholds."""
        self.cpu_thresholds = cpu_thresholds or {
            'warning': 75.0,
            'critical': 90.0,
            'extreme': 95.0
        }

        self.memory_thresholds = memory_thresholds or {
            'warning': 85.0,
            'critical': 90.0,
            'extreme': 95.0
        }

        self.disk_thresholds = disk_thresholds or {
            'warning': 90.0,
            'critical': 95.0,
            'extreme': 98.0
        }

        self.base_timestamp = time.time()
        self.log_file = Path(log_file) if log_file else None

        # Initialize logging
        self.logger = logging.getLogger(__name__)
        if self.log_file:
            handler = logging.FileHandler(self.log_file)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

        self.logger.setLevel(logging.INFO)

    def get_system_health_report(self) -> SystemHealthReport:
        """Generate a comprehensive system health report."""
        timestamp = time.time()

        # Get system metrics
        cpu_usage = self._get_cpu_usage()
        memory_usage = self._get_memory_usage()
        disk_usage = self._get_disk_usage()
        thermal_sensors = self._get_thermal_info()
        network_stats = self._get_network_stats()
        battery_status = self._get_battery_status()
        uptime = self._get_system_uptime()
        process_stats = self._get_process_stats()

        # Analyze system health
        status, warnings, critical_issues = self._analyze_system_health(
            cpu_usage, memory_usage, disk_usage, thermal_sensors
        )

        # Log any issues
        if critical_issues:
            for issue in critical_issues:
                self.logger.error(issue)
        elif warnings:
            for warning in warnings:
                self.logger.warning(warning)

        return SystemHealthReport(
            timestamp=timestamp,
            cpu_usage=cpu_usage,
            memory_usage=memory_usage,
            disk_usage=disk_usage,
            thermal_sensors=thermal_sensors,
            system_status=status,
            warnings=warnings,
            critical_issues=critical_issues,
            network_stats=network_stats,
            battery_status=battery_status,
            uptime=uptime,
            process_stats=process_stats
        )

    def _get_cpu_usage(self) -> float:
        """Get CPU usage with error handling."""
        try:
            return psutil.cpu_percent(interval=0.1)
        except Exception as e:
            self.logger.error(f"Error getting CPU usage: {e}")
            return 0.0

    def _get_memory_usage(self) -> float:
        """Get memory usage with error handling."""
        try:
            memory = psutil.virtual_memory()
            return memory.percent
        except Exception as e:
            self.logger.error(f"Error getting memory usage: {e}")
            return 0.0

    def _get_disk_usage(self) -> float:
        """Get disk usage for root partition."""
        try:
            return psutil.disk_usage('/').percent
        except Exception as e:
            self.logger.error(f"Error getting disk usage: {e}")
            return 0.0

    def _get_thermal_info(self) -> Dict[str, float]:
        """Get thermal information with error handling."""
        thermal_sensors = {}
        try:
            if hasattr(psutil, 'sensors_temperatures'):
                temps = psutil.sensors_temperatures()
                if temps:
                    for name, entries in temps.items():
                        for i, entry in enumerate(entries):
                            sensor_name = f"{name}_{i}" if len(entries) > 1 else name
                            thermal_sensors[sensor_name] = entry.current
                else:
                    thermal_sensors = {"no_sensors": -1}
            else:
                thermal_sensors = {"not_available": -1}
        except Exception as e:
            self.logger.warning(f"Error getting thermal sensors: {e}")
            thermal_sensors = {"error": -1}

        return thermal_sensors

    def _get_network_stats(self) -> Dict[str, Any]:
        """Get network statistics."""
        try:
            net_io = psutil.net_io_counters()
            return {
                'bytes_sent': net_io.bytes_sent,
                'bytes_recv': net_io.bytes_recv,
                'packets_sent': net_io.packets_sent,
                'packets_recv': net_io.packets_recv,
                'errin': net_io.errin,
                'errout': net_io.errout,
                'dropin': net_io.dropin,
                'dropout': net_io.dropout
            }
        except Exception as e:
            self.logger.error(f"Error getting network stats: {e}")
            return {}

    def _get_battery_status(self) -> Optional[Dict[str, Any]]:
        """Get battery information if available."""
        try:
            if hasattr(psutil, 'sensors_battery'):
                battery = psutil.sensors_battery()
                if battery:
                    return {
                        'percent': battery.percent,
                        'secsleft': battery.secsleft,
                        'power_plugged': battery.power_plugged
                    }
        except Exception as e:
            self.logger.warning(f"Error getting battery status: {e}")

        return None

    def _get_system_uptime(self) -> float:
        """Get system uptime in seconds."""
        try:
            boot_time = psutil.boot_time()
            return time.time() - boot_time
        except Exception as e:
            self.logger.error(f"Error getting system uptime: {e}")
            return 0.0

    def _get_process_stats(self) -> Dict[str, Any]:
        """Get process-related statistics."""
        try:
            # Count running processes
            processes = list(psutil.process_iter(['pid', 'name', 'status']))
            running_processes = [p for p in processes if p.info['status'] == 'running']
            sleeping_processes = [p for p in processes if p.info['status'] == 'sleeping']

            return {
                'total_processes': len(processes),
                'running_processes': len(running_processes),
                'sleeping_processes': len(sleeping_processes),
                'load_average': self._get_load_average()
            }
        except Exception as e:
            self.logger.error(f"Error getting process stats: {e}")
            return {}

    def _get_load_average(self) -> List[float]:
        """Get system load average (Unix/Linux only)."""
        try:
            if hasattr(os, 'getloadavg'):
                return list(os.getloadavg())
        except:
            pass
        return [0.0, 0.0, 0.0]  # Default for systems without load average

    def _analyze_system_health(self,
                              cpu_usage: float,
                              memory_usage: float,
                              disk_usage: float,
                              thermal_sensors: Dict[str, float]) -> tuple:
        """Analyze system health and identify potential issues with detailed checks."""
        warnings = []
        critical_issues = []

        # Check CPU usage against thresholds
        if cpu_usage >= self.cpu_thresholds['extreme']:
            critical_issues.append(f"CRITICAL: CPU usage is extremely high: {cpu_usage}%")
        elif cpu_usage >= self.cpu_thresholds['critical']:
            critical_issues.append(f"CRITICAL: CPU usage is very high: {cpu_usage}%")
        elif cpu_usage >= self.cpu_thresholds['warning']:
            warnings.append(f"WARNING: High CPU usage: {cpu_usage}%")

        # Check memory usage against thresholds
        if memory_usage >= self.memory_thresholds['extreme']:
            critical_issues.append(f"CRITICAL: Memory usage is extremely high: {memory_usage}%")
        elif memory_usage >= self.memory_thresholds['critical']:
            critical_issues.append(f"CRITICAL: Memory usage is very high: {memory_usage}%")
        elif memory_usage >= self.memory_thresholds['warning']:
            warnings.append(f"WARNING: High memory usage: {memory_usage}%")

        # Check disk usage against thresholds
        if disk_usage >= self.disk_thresholds['extreme']:
            critical_issues.append(f"CRITICAL: Disk usage is extremely high: {disk_usage}%")
        elif disk_usage >= self.disk_thresholds['critical']:
            critical_issues.append(f"CRITICAL: Disk usage is very high: {disk_usage}%")
        elif disk_usage >= self.disk_thresholds['warning']:
            warnings.append(f"WARNING: High disk usage: {disk_usage}%")

        # Check thermal sensors
        if thermal_sensors and all(v != -1 for v in thermal_sensors.values()):
            high_temps = [name for name, temp in thermal_sensors.items() if temp > 80.0]
            if high_temps:
                critical_issues.append(f"CRITICAL: High thermal readings: {high_temps}")
            elif any(temp > 70.0 for temp in thermal_sensors.values()):
                warnings.append(f"WARNING: Elevated thermal readings")

        # Determine overall status
        if critical_issues:
            status = "CRITICAL"
        elif warnings:
            status = "WARNING"
        else:
            status = "HEALTHY"

        return status, warnings, critical_issues

    def save_report(self, report: SystemHealthReport, filepath: str) -> None:
        """Save health report to file."""
        try:
            report_dict = {
                'timestamp': report.timestamp,
                'cpu_usage': report.cpu_usage,
                'memory_usage': report.memory_usage,
                'disk_usage': report.disk_usage,
                'thermal_sensors': report.thermal_sensors,
                'system_status': report.system_status,
                'warnings': report.warnings,
                'critical_issues': report.critical_issues,
                'network_stats': report.network_stats,
                'battery_status': report.battery_status,
                'uptime': report.uptime,
                'process_stats': report.process_stats
            }

            with open(filepath, 'w') as f:
                json.dump(report_dict, f, indent=2)

        except Exception as e:
            self.logger.error(f"Error saving report to {filepath}: {e}")


def get_system_health(cpu_thresholds: Dict[str, float] = None,
                     memory_thresholds: Dict[str, float] = None,
                     disk_thresholds: Dict[str, float] = None) -> SystemHealthReport:
    """Get current system health with optional custom thresholds."""
    monitor = SystemHealthMonitor(
        cpu_thresholds=cpu_thresholds,
        memory_thresholds=memory_thresholds,
        disk_thresholds=disk_thresholds
    )
    return monitor.get_system_health_report()


# Global instance for system-wide access
_health_monitor = None
_health_monitor_lock = threading.Lock()


def get_system_health_monitor() -> SystemHealthMonitor:
    """Get a thread-safe instance of the health monitor."""
    global _health_monitor
    with _health_monitor_lock:
        if _health_monitor is None:
            _health_monitor = SystemHealthMonitor()
    return _health_monitor


__all__ = [
    "SystemHealthReport",
    "SystemHealthMonitor",
    "get_system_health",
    "get_system_health_monitor"
]