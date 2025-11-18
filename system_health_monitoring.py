#!/usr/bin/env python3
"""
System Health Monitoring and Diagnostics for Sunnypilot
Provides comprehensive monitoring and diagnostic capabilities for the autonomous driving system.
"""

import time
import psutil
import threading
import json
import logging
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from pathlib import Path
import subprocess
import platform


@dataclass
class SystemHealthReport:
    """Dataclass for system health report."""
    timestamp: float
    cpu_usage: float
    memory_usage: Dict[str, float]
    disk_usage: Dict[str, float]
    network_usage: Dict[str, float]
    thermal_sensors: Dict[str, float]
    process_health: Dict[str, Any]
    system_status: str
    warnings: List[str]
    critical_issues: List[str]


class SystemHealthMonitor:
    """Comprehensive system health monitoring for sunnypilot."""

    def __init__(self, update_interval: float = 1.0, log_file: str = "system_health.log"):
        self.update_interval = update_interval
        self.log_file = log_file
        self.monitoring = False
        self.monitor_thread = None
        self.health_reports = []
        self.max_history = 1000
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)

    def start_monitoring(self):
        """Start continuous system health monitoring."""
        if self.monitoring:
            return

        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()
        self.logger.info("System health monitoring started.")

    def stop_monitoring(self):
        """Stop continuous system health monitoring."""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()
        self.logger.info("System health monitoring stopped.")

    def _monitoring_loop(self):
        """Main monitoring loop."""
        while self.monitoring:
            try:
                report = self.get_system_health_report()
                self.health_reports.append(report)
                
                # Keep only recent reports
                if len(self.health_reports) > self.max_history:
                    self.health_reports.pop(0)
                
                # Log critical issues
                if report.critical_issues:
                    for issue in report.critical_issues:
                        self.logger.critical(f"CRITICAL: {issue}")
                
                # Log warnings
                if report.warnings:
                    for warning in report.warnings:
                        self.logger.warning(f"WARNING: {warning}")
                        
            except Exception as e:
                self.logger.error(f"Error in monitoring loop: {e}")
                
            time.sleep(self.update_interval)

    def get_system_health_report(self) -> SystemHealthReport:
        """Generate a comprehensive system health report."""
        timestamp = time.time()
        
        # Get CPU usage
        cpu_usage = psutil.cpu_percent(interval=None)  # Non-blocking call
        cpu_per_core = psutil.cpu_percent(percpu=True)
        
        # Get memory usage
        memory = psutil.virtual_memory()
        swap_memory = psutil.swap_memory()
        
        memory_usage = {
            "total_mb": memory.total / (1024**2),
            "available_mb": memory.available / (1024**2),
            "used_mb": memory.used / (1024**2),
            "percentage": memory.percent,
            "swap_total_mb": swap_memory.total / (1024**2),
            "swap_used_mb": swap_memory.used / (1024**2),
            "swap_percentage": swap_memory.percent
        }
        
        # Get disk usage
        disk_usage = {}
        for partition in psutil.disk_partitions():
            try:
                usage = psutil.disk_usage(partition.mountpoint)
                disk_usage[partition.mountpoint] = {
                    "total_gb": usage.total / (1024**3),
                    "used_gb": usage.used / (1024**3),
                    "free_gb": usage.free / (1024**3),
                    "percentage": usage.percent
                }
            except PermissionError:
                continue  # Skip partitions that require elevated permissions
        
        # Get network usage
        net_io = psutil.net_io_counters()
        network_usage = {
            "bytes_sent": net_io.bytes_sent,
            "bytes_recv": net_io.bytes_recv,
            "packets_sent": net_io.packets_sent,
            "packets_recv": net_io.packets_recv,
            "errors_in": net_io.errin,
            "errors_out": net_io.errout
        }
        
        # Get thermal information (if available)
        thermal_sensors = {}
        try:
            temps = psutil.sensors_temperatures()
            for name, entries in temps.items():
                for entry in entries:
                    thermal_sensors[f"{name}_{entry.label or 'temp'}"] = entry.current
        except AttributeError:
            # sensors_temperatures not available on some platforms
            thermal_sensors["error"] = -1
        
        # Get process-specific information for the sunnypilot process
        process_info = self._get_process_health()
        
        # Determine system status and identify issues
        status, warnings, critical_issues = self._analyze_system_health(
            cpu_usage, memory_usage, disk_usage
        )
        
        return SystemHealthReport(
            timestamp=timestamp,
            cpu_usage=cpu_usage,
            memory_usage=memory_usage,
            disk_usage=disk_usage,
            network_usage=network_usage,
            thermal_sensors=thermal_sensors,
            process_health=process_info,
            system_status=status,
            warnings=warnings,
            critical_issues=critical_issues
        )

    def _get_process_health(self) -> Dict[str, Any]:
        """Get health information specific to sunnypilot processes."""
        process_info = {
            "current_process": {},
            "related_processes": [],
            "system_processes": []
        }
        
        try:
            # Current process information
            current_proc = psutil.Process()
            current_proc_info = {
                "pid": current_proc.pid,
                "name": current_proc.name(),
                "status": current_proc.status(),
                "cpu_percent": current_proc.cpu_percent(),
                "memory_mb": current_proc.memory_info().rss / (1024**2),
                "num_threads": current_proc.num_threads(),
                "num_fds": current_proc.num_fds() if platform.system() != "Windows" else 0,
                "io_counters": current_proc.io_counters()._asdict() if current_proc.io_counters() else {},
                "create_time": current_proc.create_time(),
                "cmdline": current_proc.cmdline()
            }
            process_info["current_process"] = current_proc_info
            
            # Related processes (find other python processes)
            related_procs = []
            for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
                try:
                    if ('python' in proc.info['name'].lower() or 
                        'sunnypilot' in proc.info['name'].lower()):
                        related_procs.append(proc.info)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            process_info["related_processes"] = related_procs[:10]  # Limit to top 10
            
            # Top CPU and memory consuming processes
            system_procs = []
            for proc in sorted(psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']), 
                             key=lambda p: p.info['cpu_percent'], reverse=True)[:5]:
                try:
                    system_procs.append(proc.info)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            process_info["system_processes"] = system_procs
            
        except Exception as e:
            process_info["error"] = str(e)
        
        return process_info

    def _analyze_system_health(self, cpu_usage: float, memory_usage: Dict[str, float], 
                              disk_usage: Dict[str, Dict[str, float]]) -> tuple:
        """Analyze system health and identify potential issues."""
        warnings = []
        critical_issues = []
        
        # Check CPU usage
        if cpu_usage > 90:
            critical_issues.append(f"CRITICAL: CPU usage is extremely high: {cpu_usage}%")
        elif cpu_usage > 70:
            warnings.append(f"WARNING: High CPU usage: {cpu_usage}%. Consider optimization.")
        
        # Check memory usage  
        mem_percent = memory_usage["percentage"]
        if mem_percent > 90:
            critical_issues.append(f"CRITICAL: Memory usage is extremely high: {mem_percent}%")
        elif mem_percent > 80:
            warnings.append(f"WARNING: High memory usage: {mem_percent}%. Monitor closely.")
        
        # Check disk usage
        for mount_point, usage in disk_usage.items():
            if usage["percentage"] > 95:
                critical_issues.append(f"CRITICAL: Disk usage on {mount_point} is extremely high: {usage['percentage']}%")
            elif usage["percentage"] > 85:
                warnings.append(f"WARNING: High disk usage on {mount_point}: {usage['percentage']}%")
        
        # Determine overall status
        if critical_issues:
            status = "CRITICAL"
        elif warnings:
            status = "WARNING"
        else:
            status = "HEALTHY"
        
        return status, warnings, critical_issues

    def get_historical_health(self, hours_back: int = 1) -> List[SystemHealthReport]:
        """Get historical health reports from the last specified hours."""
        if not self.health_reports:
            return []
        
        cutoff_time = time.time() - (hours_back * 3600)
        recent_reports = [r for r in self.health_reports if r.timestamp >= cutoff_time]
        return recent_reports

    def generate_health_summary(self, hours_back: int = 1) -> Dict[str, Any]:
        """Generate a summary of health over the past specified hours."""
        reports = self.get_historical_health(hours_back)
        
        if not reports:
            return {"error": f"No health reports available for the last {hours_back} hours"}
        
        # Calculate averages and ranges
        avg_cpu = sum(r.cpu_usage for r in reports) / len(reports)
        peak_cpu = max(r.cpu_usage for r in reports)
        
        avg_mem = sum(r.memory_usage["percentage"] for r in reports) / len(reports)
        peak_mem = max(r.memory_usage["percentage"] for r in reports)
        
        # Count issues
        warning_count = sum(len(r.warnings) for r in reports)
        critical_count = sum(len(r.critical_issues) for r in reports)
        
        # Determine overall trend
        if critical_count > 0:
            trend = "POOR"
        elif warning_count > len(reports) * 0.3:  # More than 30% of reports have warnings
            trend = "FAIR"
        else:
            trend = "GOOD"
        
        return {
            "period_hours": hours_back,
            "total_reports": len(reports),
            "average_cpu_percent": avg_cpu,
            "peak_cpu_percent": peak_cpu,
            "average_memory_percent": avg_mem,
            "peak_memory_percent": peak_mem,
            "total_warnings": warning_count,
            "total_critical_issues": critical_count,
            "overall_trend": trend,
            "timestamp_range": {
                "start": reports[0].timestamp if reports else 0,
                "end": reports[-1].timestamp if reports else 0
            }
        }

    def save_health_report(self, filename: str = None) -> str:
        """Save the latest health report to a file."""
        if not self.health_reports:
            report = self.get_system_health_report()
        else:
            report = self.health_reports[-1]
        
        if not filename:
            timestamp = int(report.timestamp)
            filename = f"system_health_report_{timestamp}.json"
        
        # Convert dataclass to dict for JSON serialization
        report_data = {
            "timestamp": report.timestamp,
            "cpu_usage": report.cpu_usage,
            "memory_usage": report.memory_usage,
            "disk_usage": report.disk_usage,
            "network_usage": report.network_usage,
            "thermal_sensors": report.thermal_sensors,
            "process_health": report.process_health,
            "system_status": report.system_status,
            "warnings": report.warnings,
            "critical_issues": report.critical_issues
        }
        
        with open(filename, 'w') as f:
            json.dump(report_data, f, indent=2, default=str)
        
        self.logger.info(f"Health report saved to: {filename}")
        return filename

    def get_health_insights(self) -> Dict[str, str]:
        """Generate insights about the system health."""
        insights = {
            "performance_recommendations": [],
            "resource_optimization": [],
            "critical_alerts": []
        }
        
        # Get recent health reports
        recent_reports = self.get_historical_health(hours_back=1)
        if not recent_reports:
            return insights
        
        # Analyze trends
        avg_cpu = sum(r.cpu_usage for r in recent_reports) / len(recent_reports)
        avg_mem = sum(r.memory_usage["percentage"] for r in recent_reports) / len(recent_reports)
        
        # Performance recommendations
        if avg_cpu > 60:
            insights["performance_recommendations"].append(
                "Average CPU usage is high (>60%). Consider optimizing performance-critical code paths."
            )
        if avg_mem > 70:
            insights["performance_recommendations"].append(
                "Average memory usage is high (>70%). Consider memory optimization strategies."
            )
        
        # Resource optimization
        if avg_cpu > 50:
            insights["resource_optimization"].append(
                "Consider ARM NEON optimizations for CPU-intensive operations."
            )
        if avg_mem > 60:
            insights["resource_optimization"].append(
                "Consider memory pooling and object reuse to reduce allocation overhead."
            )
        
        # Check for critical alerts from recent reports
        for report in recent_reports[-10:]:  # Check last 10 reports
            insights["critical_alerts"].extend(report.critical_issues)
        
        # Remove duplicates
        insights["performance_recommendations"] = list(set(insights["performance_recommendations"]))
        insights["resource_optimization"] = list(set(insights["resource_optimization"]))
        insights["critical_alerts"] = list(set(insights["critical_alerts"]))
        
        return insights


def run_system_health_check():
    """Run a comprehensive system health check."""
    print("Sunnypilot System Health Check")
    print("==============================")
    
    monitor = SystemHealthMonitor(update_interval=2.0)  # Update every 2 seconds during check
    
    print("Performing system health analysis...")
    
    # Get current system health
    report = monitor.get_system_health_report()
    
    print(f"\nSystem Status: {report.system_status}")
    print(f"Timestamp: {time.ctime(report.timestamp)}")
    print(f"CPU Usage: {report.cpu_usage}%")
    print(f"Memory Usage: {report.memory_usage['percentage']:.1f}%")
    print(f"Memory Used: {report.memory_usage['used_mb']:.1f} MB")
    
    # Print disk usage
    print(f"\nDisk Usage:")
    for mount_point, usage in report.disk_usage.items():
        print(f"  {mount_point}: {usage['percentage']:.1f}% ({usage['used_gb']:.1f} GB used)")
    
    # Print thermal info if available
    if report.thermal_sensors and "error" not in report.thermal_sensors:
        print(f"\nThermal Sensors:")
        for sensor, temp in report.thermal_sensors.items():
            print(f"  {sensor}: {temp}°C")
    else:
        print(f"\nThermal sensors: Not available on this platform")
    
    # Print warnings and critical issues
    if report.warnings:
        print(f"\nWarnings ({len(report.warnings)}):")
        for warning in report.warnings:
            print(f"  ⚠️  {warning}")
    
    if report.critical_issues:
        print(f"\nCritical Issues ({len(report.critical_issues)}):")
        for issue in report.critical_issues:
            print(f"  ❌ {issue}")
    
    # Generate insights
    insights = monitor.get_health_insights()
    print(f"\nInsights:")
    if insights["performance_recommendations"]:
        print(f"  Performance Recommendations ({len(insights['performance_recommendations'])}):")
        for rec in insights["performance_recommendations"]:
            print(f"    • {rec}")
    
    if insights["resource_optimization"]:
        print(f"  Resource Optimization ({len(insights['resource_optimization'])}):")
        for opt in insights["resource_optimization"]:
            print(f"    • {opt}")
    
    if insights["critical_alerts"]:
        print(f"  Previous Critical Alerts ({len(insights['critical_alerts'])}):")
        for alert in insights["critical_alerts"]:
            print(f"    • {alert}")
    
    # Save report
    filename = monitor.save_health_report()
    print(f"\nDetailed health report saved to: {filename}")
    
    return report


def main():
    """Run the system health monitoring and diagnostics."""
    print("Sunnypilot System Health Monitoring & Diagnostics")
    print("=================================================")
    
    # Run immediate health check
    report = run_system_health_check()
    
    # If no critical issues, suggest starting continuous monitoring
    if not report.critical_issues:
        print(f"\nNo critical issues detected. Starting continuous monitoring...")
        print("Press Ctrl+C to stop monitoring.")
        
        monitor = SystemHealthMonitor(update_interval=5.0)
        monitor.start_monitoring()
        
        try:
            while True:
                time.sleep(10)  # Pause for 10 seconds
        except KeyboardInterrupt:
            print(f"\nStopping continuous monitoring...")
            monitor.stop_monitoring()
            print("Continuous monitoring stopped.")
    
    return report


if __name__ == "__main__":
    main()