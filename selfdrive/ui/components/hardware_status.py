"""
Hardware Status Dashboard Component
Real-time visualization of CPU/RAM/Power usage with efficient rendering
"""
import pyray as rl
import time
from typing import Dict, Tuple
from dataclasses import dataclass

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.sunnypilot_ui import UIComponent


@dataclass
class HardwareMetrics:
    """Container for hardware metrics"""
    cpu_usage: float = 0.0
    cpu_usage_per_core: list = None
    ram_usage: float = 0.0
    ram_total: float = 0.0
    ram_used: float = 0.0
    power_draw: float = 0.0
    temperature: float = 0.0
    uptime: float = 0.0
    
    def __post_init__(self):
        if self.cpu_usage_per_core is None:
            self.cpu_usage_per_core = [0.0] * 4  # Assume 4 cores for Comma Three


class HardwareStatusDashboard(UIComponent):
    """Hardware Status Dashboard showing real-time system metrics"""
    
    def __init__(self):
        super().__init__("HardwareStatusDashboard")
        
        # Metrics
        self.metrics = HardwareMetrics()
        self.metrics_history = {
            'cpu': [],
            'ram': [],
            'power': [],
        }
        self.max_history_points = 100  # For graph display
        
        # UI Elements
        self.gauge_radius = 40
        self.panel_width = 250
        self.panel_height = 300
        self.panel_x = 20
        self.panel_y = 20
        
        # Thresholds for color coding
        self.cpu_warning_threshold = 75.0
        self.cpu_critical_threshold = 90.0
        self.ram_warning_threshold = 80.0
        self.ram_critical_threshold = 90.0
        self.temp_warning_threshold = 70.0
        self.temp_critical_threshold = 85.0
        self.power_warning_threshold = 8.0
        self.power_critical_threshold = 9.5
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update hardware metrics from system messages"""
        if sm.updated["deviceState"]:
            device_state = sm["deviceState"]
            
            # Update metrics
            self.metrics.cpu_usage = device_state.cpuPercent
            self.metrics.ram_usage = device_state.memoryPercent
            self.metrics.power_draw = device_state.powerDraw
            self.metrics.temperature = device_state.cpuTempC
            
            # Update CPU per core (simplified - in reality this comes from multiple sources)
            if hasattr(device_state, 'cpuCores'):
                for i, core_usage in enumerate(device_state.cpuCores):
                    if i < len(self.metrics.cpu_usage_per_core):
                        self.metrics.cpu_usage_per_core[i] = core_usage
            else:
                # Fallback: distribute total CPU usage across cores
                avg_core_usage = device_state.cpuPercent / len(self.metrics.cpu_usage_per_core)
                self.metrics.cpu_usage_per_core = [avg_core_usage] * 4
            
            # Update history for graphing
            self._update_history()
    
    def _update_history(self):
        """Update metric history for graphing"""
        self.metrics_history['cpu'].append(self.metrics.cpu_usage)
        self.metrics_history['ram'].append(self.metrics.ram_usage)
        self.metrics_history['power'].append(self.metrics.power_draw)
        
        # Limit history to max points
        for key in self.metrics_history:
            if len(self.metrics_history[key]) > self.max_history_points:
                self.metrics_history[key] = self.metrics_history[key][-self.max_history_points:]
    
    def _get_gauge_color(self, value: float, warning_threshold: float, critical_threshold: float) -> rl.Color:
        """Get color for gauge based on value thresholds"""
        if value >= critical_threshold:
            return rl.Color(220, 20, 60, 255)  # Red for critical
        elif value >= warning_threshold:
            return rl.Color(255, 165, 0, 255)  # Orange for warning
        else:
            return rl.Color(50, 205, 50, 255)  # Green for normal
    
    def _render_gauge(self, center_x: float, center_y: float, radius: float, 
                     value: float, max_value: float, label: str):
        """Render a circular gauge for metrics"""
        # Draw background circle
        rl.draw_circle_lines(int(center_x), int(center_y), radius, rl.Color(100, 100, 110, 200))
        
        # Draw filled arc based on value
        fill_percent = min(1.0, value / max_value)
        angle = fill_percent * 360.0
        
        # Get gauge color based on value
        color = self._get_gauge_color(
            value, 
            self.cpu_warning_threshold if "CPU" in label else self.ram_warning_threshold if "RAM" in label else self.temp_warning_threshold,
            self.cpu_critical_threshold if "CPU" in label else self.ram_critical_threshold if "RAM" in label else self.temp_critical_threshold
        )
        
        # Draw arc
        for i in range(int(angle)):
            angle_rad = (i / 360.0) * 2 * 3.14159
            x = center_x + radius * 0.8 * rl.cos(angle_rad)
            y = center_y + radius * 0.8 * rl.sin(angle_rad)
            rl.draw_pixel(int(x), int(y), color)
        
        # Draw value text
        text = f"{value:.1f}"
        text_width = rl.measure_text(text, 20)
        rl.draw_text(text, int(center_x - text_width / 2), int(center_y - 10), 20, rl.WHITE)
        rl.draw_text(label, int(center_x - rl.measure_text(label, 14) / 2), int(center_y + 15), 14, rl.GRAY)
    
    def _render_progress_bar(self, x: float, y: float, width: float, height: float,
                           value: float, max_value: float, label: str):
        """Render a horizontal progress bar"""
        # Draw background
        rl.draw_rectangle(int(x), int(y), int(width), int(height), rl.Color(50, 50, 60, 200))
        
        # Calculate fill width
        fill_width = (value / max_value) * width if max_value > 0 else 0
        fill_width = min(fill_width, width)
        
        # Get bar color based on value
        color = self._get_gauge_color(value, 75.0, 90.0)
        rl.draw_rectangle(int(x), int(y), int(fill_width), int(height), color)
        
        # Draw border
        rl.draw_rectangle_lines(int(x), int(y), int(width), int(height), rl.Color(150, 150, 160, 200))
        
        # Draw value text
        rl.draw_text(f"{label}: {value:.1f}", int(x), int(y - 20), 16, rl.WHITE)
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render the hardware status dashboard"""
        if not self.visible:
            return
        
        # Draw dashboard panel background
        bg_color = rl.Color(30, 30, 40, 220)  # Semi-transparent dark background
        border_color = rl.Color(100, 100, 120, 255)
        
        rl.draw_rectangle(self.panel_x, self.panel_y, self.panel_width, self.panel_height, bg_color)
        rl.draw_rectangle_lines(self.panel_x, self.panel_y, self.panel_width, self.panel_height, border_color)
        
        # Draw title
        rl.draw_text("HARDWARE STATUS", self.panel_x + 10, self.panel_y + 5, 18, rl.LIGHTGRAY)
        
        # Calculate positions for gauges
        row1_y = self.panel_y + 40
        row2_y = self.panel_y + 150
        gauge_x1 = self.panel_x + 60
        gauge_x2 = self.panel_x + 170
        
        # Render CPU gauge
        self._render_gauge(gauge_x1, row1_y, self.gauge_radius, 
                          self.metrics.cpu_usage, 100.0, "CPU %")
        
        # Render RAM gauge
        self._render_gauge(gauge_x2, row1_y, self.gauge_radius, 
                          self.metrics.ram_usage, 100.0, "RAM %")
        
        # Render temperature gauge
        self._render_gauge(gauge_x1, row2_y, self.gauge_radius, 
                          self.metrics.temperature, 100.0, "TEMP C")
        
        # Render power progress bar
        power_bar_x = self.panel_x + 20
        power_bar_y = row2_y + 60
        self._render_progress_bar(power_bar_x, power_bar_y, 200, 15, 
                                 self.metrics.power_draw, 10.0, "POWER")
        
        # Draw additional metrics text
        metrics_text_y = self.panel_y + 250
        rl.draw_text(f"Uptime: {self.metrics.uptime:.1f}s", self.panel_x + 10, metrics_text_y, 14, rl.LIGHTGRAY)


class CompactHardwareStatus(UIComponent):
    """Compact version of hardware status for minimal UI display"""
    
    def __init__(self):
        super().__init__("CompactHardwareStatus")
        
        # Metrics
        self.cpu_usage = 0.0
        self.ram_usage = 0.0
        self.power_draw = 0.0
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update with minimal metrics"""
        if sm.updated["deviceState"]:
            device_state = sm["deviceState"]
            self.cpu_usage = device_state.cpuPercent
            self.ram_usage = device_state.memoryPercent
            self.power_draw = device_state.powerDraw
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render compact hardware status in corner"""
        # Draw small metrics in bottom-left corner
        x = rect.x + 10
        y = rect.y + rect.height - 60
        width = 160
        height = 50
        
        # Semi-transparent background
        bg_color = rl.Color(20, 20, 30, 180)
        rl.draw_rectangle(int(x), int(y), int(width), int(height), bg_color)
        rl.draw_rectangle_lines(int(x), int(y), int(width), int(height), rl.Color(100, 100, 120, 200))
        
        # Draw metrics text
        text_color = rl.Color(200, 220, 255, 255)
        rl.draw_text(f"CPU:{self.cpu_usage:.0f}%", int(x + 5), int(y + 5), 16, text_color)
        rl.draw_text(f"RAM:{self.ram_usage:.0f}%", int(x + 5), int(y + 25), 16, text_color)
        
        # Color code power based on usage
        power_color = rl.Color(100, 255, 100, 255)  # Green
        if self.power_draw > 8.0:
            power_color = rl.Color(255, 165, 0, 255)  # Orange
        if self.power_draw > 9.5:
            power_color = rl.Color(255, 100, 100, 255)  # Red
        rl.draw_text(f"PWR:{self.power_draw:.1f}W", int(x + 85), int(y + 5), 16, power_color)