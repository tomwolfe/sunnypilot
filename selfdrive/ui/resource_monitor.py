#!/usr/bin/env python3
"""
Resource Monitoring and Reporting for Sunnypilot UI System
Provides detailed monitoring of CPU, GPU, memory, power, and thermal usage
"""
import time
import psutil
import threading
from typing import Dict, List, Callable, Optional
from dataclasses import dataclass
from collections import deque

import pyray as rl
from cereal import messaging


@dataclass
class ResourceSnapshot:
    """Snapshot of system resources at a point in time"""
    timestamp: float
    cpu_percent: float
    cpu_per_core: List[float]
    memory_percent: float
    memory_used: float  # in MB
    memory_total: float  # in MB
    power_draw: float  # in W
    thermal_status: int
    gpu_percent: Optional[float] = None  # GPU utilization if available
    gpu_memory: Optional[float] = None  # GPU memory if available
    thermal_temps: Optional[Dict[str, float]] = None  # Temperature by sensor


class ResourceMonitor:
    """Monitors system resources with historical tracking"""
    
    def __init__(self, history_size: int = 300):  # 5 minutes at 1Hz
        self.history_size = history_size
        self.resource_history = deque(maxlen=history_size)
        self.current_resources = ResourceSnapshot(
            timestamp=0,
            cpu_percent=0,
            cpu_per_core=[],
            memory_percent=0,
            memory_used=0,
            memory_total=0,
            power_draw=0,
            thermal_status=0
        )
        
        # Message subscriber for device state
        self.sm = messaging.SubMaster(["deviceState"])
        
        # Monitoring thread
        self.monitoring = False
        self.monitoring_thread = None
        
        # Callbacks for resource alerts
        self.resource_alert_callbacks: List[Callable] = []
        self.cpu_threshold = 80.0  # percent
        self.memory_threshold = 85.0  # percent
        self.temp_threshold = 80.0  # celsius
        
    def start_monitoring(self):
        """Start the resource monitoring thread"""
        if self.monitoring:
            return
            
        self.monitoring = True
        self.monitoring_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitoring_thread.start()
    
    def stop_monitoring(self):
        """Stop the resource monitoring thread"""
        self.monitoring = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=2.0)
    
    def _monitor_loop(self):
        """Main monitoring loop running in a separate thread"""
        while self.monitoring:
            try:
                self._update_resources()
                time.sleep(1.0)  # Update once per second
            except Exception as e:
                print(f"Error in resource monitoring: {e}")
    
    def _update_resources(self):
        """Update resource measurements"""
        # Update messaging to get latest device state
        self.sm.update(0)
        
        timestamp = time.time()
        
        # Get system-level metrics
        cpu_percent = psutil.cpu_percent()
        cpu_per_core = psutil.cpu_percent(percpu=True)
        memory = psutil.virtual_memory()
        
        # Get device state metrics if available
        power_draw = 0.0
        thermal_status = 0
        thermal_temps = {}
        
        if self.sm.updated["deviceState"]:
            device_state = self.sm["deviceState"]
            power_draw = device_state.powerDraw
            thermal_status = device_state.thermalStatus
            thermal_temps = {
                "cpu": device_state.cpuTempC,
                "gpu": device_state.gpuTempC,
                "bat": device_state.batTempC,
            }
        
        # Create new snapshot
        snapshot = ResourceSnapshot(
            timestamp=timestamp,
            cpu_percent=cpu_percent,
            cpu_per_core=cpu_per_core,
            memory_percent=memory.percent,
            memory_used=memory.used / (1024**2),  # Convert to MB
            memory_total=memory.total / (1024**2),  # Convert to MB
            power_draw=power_draw,
            thermal_status=thermal_status,
            thermal_temps=thermal_temps
        )
        
        # Store current and add to history
        self.current_resources = snapshot
        self.resource_history.append(snapshot)
        
        # Check for resource alerts
        self._check_resource_alerts(snapshot)
    
    def _check_resource_alerts(self, snapshot: ResourceSnapshot):
        """Check if any resource thresholds have been exceeded"""
        alerts = []
        
        if snapshot.cpu_percent > self.cpu_threshold:
            alerts.append(f"CPU usage high: {snapshot.cpu_percent:.1f}%")
        
        if snapshot.memory_percent > self.memory_threshold:
            alerts.append(f"Memory usage high: {snapshot.memory_percent:.1f}%")
        
        if snapshot.thermal_temps and snapshot.thermal_temps.get("cpu", 0) > self.temp_threshold:
            alerts.append(f"CPU temperature high: {snapshot.thermal_temps['cpu']:.1f}C")
        
        # Call registered alert callbacks
        for callback in self.resource_alert_callbacks:
            callback(alerts)
    
    def add_resource_alert_callback(self, callback: Callable):
        """Add a callback to be called when resource thresholds are exceeded"""
        self.resource_alert_callbacks.append(callback)
    
    def get_current_resources(self) -> ResourceSnapshot:
        """Get the most recent resource snapshot"""
        return self.current_resources
    
    def get_resource_history(self, minutes: int = 1) -> List[ResourceSnapshot]:
        """Get resource history for the last N minutes (default 1 minute = 60 samples)"""
        required_samples = min(minutes * 60, len(self.resource_history))
        return list(self.resource_history)[-required_samples:]
    
    def get_average_resources(self, minutes: int = 1) -> ResourceSnapshot:
        """Get average resources over the last N minutes"""
        history = self.get_resource_history(minutes)
        if not history:
            return self.current_resources
        
        # Calculate averages
        avg_cpu = sum(s.cpu_percent for s in history) / len(history)
        avg_memory = sum(s.memory_percent for s in history) / len(history)
        avg_power = sum(s.power_draw for s in history) / len(history)
        latest = history[-1]  # For thermal status and other non-averaged values
        
        return ResourceSnapshot(
            timestamp=time.time(),
            cpu_percent=avg_cpu,
            cpu_per_core=latest.cpu_per_core,  # Can't average per-core
            memory_percent=avg_memory,
            memory_used=latest.memory_used,
            memory_total=latest.memory_total,
            power_draw=avg_power,
            thermal_status=latest.thermal_status,
            thermal_temps=latest.thermal_temps
        )
    
    def get_peak_resources(self, minutes: int = 1) -> ResourceSnapshot:
        """Get peak resources over the last N minutes"""
        history = self.get_resource_history(minutes)
        if not history:
            return self.current_resources
        
        peak_cpu = max(s.cpu_percent for s in history)
        peak_memory = max(s.memory_percent for s in history)
        peak_power = max(s.power_draw for s in history)
        latest = history[-1]
        
        # Find the snapshot with peak values to return its associated data
        peak_cpu_snap = next(s for s in history if s.cpu_percent == peak_cpu)
        peak_memory_snap = next(s for s in history if s.memory_percent == peak_memory)
        peak_power_snap = next(s for s in history if s.power_draw == peak_power)
        
        return ResourceSnapshot(
            timestamp=time.time(),
            cpu_percent=peak_cpu,
            cpu_per_core=peak_cpu_snap.cpu_per_core,
            memory_percent=peak_memory,
            memory_used=peak_memory_snap.memory_used,
            memory_total=peak_memory_snap.memory_total,
            power_draw=peak_power,
            thermal_status=latest.thermal_status,
            thermal_temps=latest.thermal_temps
        )


class ResourceOverlay:
    """Provides an overlay for displaying resource usage"""
    
    def __init__(self, monitor: ResourceMonitor):
        self.monitor = monitor
        self.enabled = False
        self.position = rl.Vector2(10, 10)  # Top-left by default
        self.size = rl.Vector2(250, 200)  # Default size
        
    def toggle_enabled(self):
        """Toggle the overlay on/off"""
        self.enabled = not self.enabled
    
    def render(self, rect: rl.Rectangle):
        """Render the resource overlay"""
        if not self.enabled:
            return
        
        # Get current resource snapshot
        resources = self.monitor.get_current_resources()
        
        # Draw overlay background
        bg_color = rl.Color(20, 20, 30, 180)  # Dark semi-transparent
        border_color = rl.Color(100, 100, 120, 200)  # Muted border
        text_color = rl.Color(200, 220, 255, 255)  # Light blue text
        
        # Draw background panel
        rl.draw_rectangle(int(self.position.x), int(self.position.y), 
                         int(self.size.x), int(self.size.y), bg_color)
        rl.draw_rectangle_lines(int(self.position.x), int(self.position.y), 
                              int(self.size.x), int(self.size.y), border_color)
        
        # Draw title
        rl.draw_text("SYSTEM RESOURCES", int(self.position.x + 10), 
                    int(self.position.y + 5), 16, rl.LIGHTGRAY)
        
        # Draw resource information
        y_offset = 30
        line_height = 20
        
        # CPU usage
        cpu_color = rl.Color(100, 255, 100, 255) if resources.cpu_percent < 70 else \
                   rl.Color(255, 200, 0, 255) if resources.cpu_percent < 90 else \
                   rl.Color(255, 100, 100, 255)
        rl.draw_text(f"CPU: {resources.cpu_percent:.1f}%", 
                    int(self.position.x + 10), int(self.position.y + y_offset), 
                    14, cpu_color)
        y_offset += line_height
        
        # Memory usage
        mem_color = rl.Color(100, 255, 100, 255) if resources.memory_percent < 70 else \
                   rl.Color(255, 200, 0, 255) if resources.memory_percent < 90 else \
                   rl.Color(255, 100, 100, 255)
        rl.draw_text(f"RAM: {resources.memory_percent:.1f}% ({resources.memory_used:.0f}MB)", 
                    int(self.position.x + 10), int(self.position.y + y_offset), 
                    14, mem_color)
        y_offset += line_height
        
        # Power draw
        pow_color = rl.Color(100, 255, 100, 255) if resources.power_draw < 7.0 else \
                   rl.Color(255, 200, 0, 255) if resources.power_draw < 9.0 else \
                   rl.Color(255, 100, 100, 255)
        rl.draw_text(f"Power: {resources.power_draw:.2f}W", 
                    int(self.position.x + 10), int(self.position.y + y_offset), 
                    14, pow_color)
        y_offset += line_height
        
        # Thermal status (if available)
        if resources.thermal_temps:
            temp = resources.thermal_temps.get("cpu", 0)
            temp_color = rl.Color(100, 255, 100, 255) if temp < 70 else \
                        rl.Color(255, 200, 0, 255) if temp < 85 else \
                        rl.Color(255, 100, 100, 255)
            rl.draw_text(f"CPU Temp: {temp:.1f}C", 
                        int(self.position.x + 10), int(self.position.y + y_offset), 
                        14, temp_color)
            y_offset += line_height
        
        # Thermal status level
        thermal_text = {0: "Normal", 1: "Green", 2: "Yellow", 3: "Red", 4: "Danger", 5: "Immediate"}
        rl.draw_text(f"Thermal: {thermal_text.get(resources.thermal_status, 'Unknown')}", 
                    int(self.position.x + 10), int(self.position.y + y_offset), 
                    14, text_color)


class ResourceReportGenerator:
    """Generates detailed resource reports"""
    
    def __init__(self, monitor: ResourceMonitor):
        self.monitor = monitor
    
    def generate_report(self, duration_minutes: int = 5) -> str:
        """Generate a detailed resource report"""
        history = self.monitor.get_resource_history(duration_minutes)
        if not history:
            return "No resource data available"
        
        avg_resources = self.monitor.get_average_resources(duration_minutes)
        peak_resources = self.monitor.get_peak_resources(duration_minutes)
        
        report = []
        report.append("=" * 60)
        report.append("SUNNYPilot UI Resource Usage Report")
        report.append("=" * 60)
        report.append(f"Reporting Period: Last {duration_minutes} minutes")
        report.append(f"Total Samples: {len(history)}")
        report.append("")
        
        report.append("AVERAGE VALUES:")
        report.append(f"  CPU Usage:      {avg_resources.cpu_percent:.2f}%")
        report.append(f"  Memory Usage:   {avg_resources.memory_percent:.2f}%")
        report.append(f"  Power Draw:     {avg_resources.power_draw:.2f}W")
        report.append("")
        
        report.append("PEAK VALUES:")
        report.append(f"  CPU Usage:      {peak_resources.cpu_percent:.2f}%")
        report.append(f"  Memory Usage:   {peak_resources.memory_percent:.2f}%")
        report.append(f"  Power Draw:     {peak_resources.power_draw:.2f}W")
        report.append("")
        
        report.append("CURRENT VALUES:")
        current = self.monitor.get_current_resources()
        report.append(f"  CPU Usage:      {current.cpu_percent:.2f}%")
        report.append(f"  Memory Usage:   {current.memory_percent:.2f}%")
        report.append(f"  Power Draw:     {current.power_draw:.2f}W")
        report.append(f"  Thermal Status: {current.thermal_status}")
        if current.thermal_temps:
            for sensor, temp in current.thermal_temps.items():
                report.append(f"  {sensor.title()} Temp:      {temp:.2f}C")
        report.append("")
        
        # Resource assessment
        report.append("ASSESSMENT:")
        issues = []
        if avg_resources.cpu_percent > 70:
            issues.append(f"High average CPU usage ({avg_resources.cpu_percent:.1f}%)")
        if avg_resources.memory_percent > 80:
            issues.append(f"High average memory usage ({avg_resources.memory_percent:.1f}%)")
        if peak_resources.cpu_percent > 90:
            issues.append(f"Very high peak CPU usage ({peak_resources.cpu_percent:.1f}%)")
        if peak_resources.memory_percent > 90:
            issues.append(f"Very high peak memory usage ({peak_resources.memory_percent:.1f}%)")
        
        if issues:
            report.append("  Issues detected:")
            for issue in issues:
                report.append(f"    - {issue}")
        else:
            report.append("  No significant resource issues detected")
        
        report.append("=" * 60)
        
        return "\n".join(report)


def main():
    """Main function to demonstrate resource monitoring"""
    print("Sunnypilot Resource Monitoring System")
    
    # Create and start monitor
    monitor = ResourceMonitor()
    monitor.start_monitoring()
    
    try:
        print("Resource monitoring started. Press Ctrl+C to stop.")
        
        # Print updates periodically
        for i in range(10):
            time.sleep(3)
            resources = monitor.get_current_resources()
            print(f"Time: {time.strftime('%H:%M:%S', time.localtime(resources.timestamp))} | "
                  f"CPU: {resources.cpu_percent:.1f}% | "
                  f"RAM: {resources.memory_percent:.1f}% | "
                  f"Power: {resources.power_draw:.2f}W")
        
        # Generate a report
        print("\nGenerating resource report...")
        report_gen = ResourceReportGenerator(monitor)
        report = report_gen.generate_report(1)  # 1 minute report
        print("\n" + report)
        
    except KeyboardInterrupt:
        print("\nStopping resource monitoring...")
    finally:
        monitor.stop_monitoring()
        print("Resource monitoring stopped.")


if __name__ == "__main__":
    main()