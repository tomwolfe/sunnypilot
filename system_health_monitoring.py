#!/usr/bin/env python3
"""
System Health Monitoring for Sunnypilot
Provides system health monitoring for the autonomous driving system.
"""

import time
import psutil
from typing import Dict, Any, List
from dataclasses import dataclass


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


class SystemHealthMonitor:
    """Simple system health monitoring for sunnypilot."""

    def get_system_health_report(self) -> SystemHealthReport:
        """Generate a system health report."""
        timestamp = time.time()

        # Get CPU usage
        cpu_usage = psutil.cpu_percent(interval=None)

        # Get memory usage
        memory = psutil.virtual_memory()
        memory_usage = memory.percent

        # Get disk usage (for root partition)
        disk_usage = psutil.disk_usage('/').percent

        # Get thermal information (if available)
        thermal_sensors = {}
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for name, entries in list(temps.items())[:1]:  # Just get first sensor group
                    for entry in entries[:1]:  # Just get first sensor
                        thermal_sensors[f"{name}"] = entry.current
        except AttributeError:
            # sensors_temperatures not available on some platforms
            thermal_sensors = {"not_available": -1}

        # Determine system status and identify issues
        status, warnings, critical_issues = self._analyze_system_health(
            cpu_usage, memory_usage, disk_usage
        )

        return SystemHealthReport(
            timestamp=timestamp,
            cpu_usage=cpu_usage,
            memory_usage=memory_usage,
            disk_usage=disk_usage,
            thermal_sensors=thermal_sensors,
            system_status=status,
            warnings=warnings,
            critical_issues=critical_issues
        )

    def _analyze_system_health(self, cpu_usage: float, memory_usage: float,
                              disk_usage: float) -> tuple:
        """Analyze system health and identify potential issues."""
        warnings = []
        critical_issues = []

        # Check CPU usage
        if cpu_usage > 95:
            critical_issues.append(f"CRITICAL: CPU usage is extremely high: {cpu_usage}%")
        elif cpu_usage > 90:
            critical_issues.append(f"CRITICAL: CPU usage is very high: {cpu_usage}%")
        elif cpu_usage > 75:
            warnings.append(f"WARNING: High CPU usage: {cpu_usage}%")

        # Check memory usage
        if memory_usage > 95:
            critical_issues.append(f"CRITICAL: Memory usage is extremely high: {memory_usage}%")
        elif memory_usage > 90:
            critical_issues.append(f"CRITICAL: Memory usage is very high: {memory_usage}%")
        elif memory_usage > 85:
            warnings.append(f"WARNING: High memory usage: {memory_usage}%")

        # Check disk usage
        if disk_usage > 98:
            critical_issues.append(f"CRITICAL: Disk usage is extremely high: {disk_usage}%")
        elif disk_usage > 95:
            critical_issues.append(f"CRITICAL: Disk usage is very high: {disk_usage}%")
        elif disk_usage > 90:
            warnings.append(f"WARNING: High disk usage: {disk_usage}%")

        # Determine overall status
        if critical_issues:
            status = "CRITICAL"
        elif warnings:
            status = "WARNING"
        else:
            status = "HEALTHY"

        return status, warnings, critical_issues


def get_system_health() -> SystemHealthReport:
    """Get current system health."""
    monitor = SystemHealthMonitor()
    return monitor.get_system_health_report()


__all__ = [
    "SystemHealthReport", "get_system_health"
]