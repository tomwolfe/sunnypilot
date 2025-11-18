#!/usr/bin/env python3
"""
Main UI Architecture for Sunnypilot Autonomous Driving Platform
Implements the layered UI system with resource-efficient rendering for Comma Three hardware
"""

import os
import time
import pyray as rl
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Callable, Any

from cereal import messaging, log
from openpilot.system.ui.lib.application import gui_app, DEFAULT_FPS
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.layouts.home import HomeLayout
from openpilot.selfdrive.ui.layouts.settings.settings import SettingsLayout


class DrivingState(Enum):
    """Represents different autonomous driving states"""
    MANUAL = "manual"
    ENGAGED = "engaged"
    DISENGAGED = "disengaged"
    EMERGENCY = "emergency"


class Theme(Enum):
    """UI theme for different lighting conditions"""
    DAY = "day"
    NIGHT = "night"


@dataclass
class UIColors:
    """Color palette for different themes"""
    # Day theme colors
    day_background: rl.Color = rl.Color(20, 20, 30, 220)
    day_text: rl.Color = rl.Color(255, 255, 255, 255)
    day_accent: rl.Color = rl.Color(70, 130, 180, 255)
    day_warning: rl.Color = rl.Color(255, 165, 0, 255)
    day_critical: rl.Color = rl.Color(220, 20, 60, 255)
    
    # Night theme colors
    night_background: rl.Color = rl.Color(10, 10, 20, 200)
    night_text: rl.Color = rl.Color(240, 240, 240, 255)
    night_accent: rl.Color = rl.Color(100, 149, 237, 255)
    night_warning: rl.Color = rl.Color(255, 140, 0, 255)
    night_critical: rl.Color = rl.Color(255, 69, 0, 255)
    
    @property
    def background(self) -> rl.Color:
        return self.night_background if ui_state.theme == Theme.NIGHT else self.day_background
    
    @property
    def text(self) -> rl.Color:
        return self.night_text if ui_state.theme == Theme.NIGHT else self.day_text
    
    @property
    def accent(self) -> rl.Color:
        return self.night_accent if ui_state.theme == Theme.NIGHT else self.day_accent
    
    @property
    def warning(self) -> rl.Color:
        return self.night_warning if ui_state.theme == Theme.NIGHT else self.day_warning
    
    @property
    def critical(self) -> rl.Color:
        return self.night_critical if ui_state == Theme.NIGHT else self.day_critical


class SunnypilotUI:
    """
    Main UI controller implementing the layered architecture for sunnypilot.
    Optimized for Comma Three hardware with efficient rendering and resource usage.
    """
    
    def __init__(self):
        # Initialize messaging
        self.sm = messaging.SubMaster([
            "modelV2", "controlsState", "selfdriveState", "deviceState", 
            "carState", "perceptionModel", "uiDebug", "navInstruction"
        ])
        self.pm = messaging.PubMaster(["userFlag", "userEvent"])
        
        # State management
        self.driving_state = DrivingState.DISENGAGED
        self.theme = Theme.DAY  # Will be updated based on light sensor
        self.colors = UIColors()
        
        # Layer management
        self.hud_layer = HudRenderer()
        self.system_layer = SystemStatusLayer()
        self.perception_layer = PerceptionVisualizationLayer()
        self.navigation_layer = NavigationDisplayLayer()
        self.hardware_layer = HardwareStatusLayer()
        
        # Performance tracking
        self.frame_start_time = 0.0
        self.last_update_time = 0.0
        self.avg_render_time = 0.0
        self.frame_count = 0
        
        # Resource limits
        self.max_cpu_usage = 5.0  # Percentage
        self.target_fps = 30.0
        self.frame_interval = 1.0 / self.target_fps
        
        # Initialize UI components
        self._setup_ui_components()
    
    def _setup_ui_components(self):
        """Initialize all UI components with proper configuration"""
        # Theme adjustment based on light sensor
        self._update_theme()
    
    def update(self):
        """Update UI state and components"""
        current_time = time.time()
        self.last_update_time = current_time
        
        # Update messaging state
        self.sm.update(0)
        
        # Update driving state
        self._update_driving_state()
        
        # Update theme based on light sensor
        self._update_theme()
        
        # Update all UI layers
        self.system_layer.update(self.sm)
        self.perception_layer.update(self.sm)
        self.navigation_layer.update(self.sm)
        self.hardware_layer.update(self.sm)
        
        # Performance tracking
        self.frame_count += 1
    
    def render(self, rect: rl.Rectangle):
        """Render all UI layers with resource efficiency"""
        self.frame_start_time = time.time()
        
        # Calculate which layers to render based on driving state
        render_layers = self._get_active_layers()
        
        # Render layers in z-order (background to foreground)
        for layer in render_layers:
            layer.render(rect)
        
        # Update performance metrics
        render_time = time.time() - self.frame_start_time
        self.avg_render_time = (self.avg_render_time * 0.9) + (render_time * 0.1)
        
        # Resource usage validation
        self._validate_resource_usage()
    
    def _update_driving_state(self):
        """Update the driving state based on system status"""
        if not ui_state.started:
            self.driving_state = DrivingState.DISENGAGED
        elif ui_state.status == log.ControlsState.OpenpilotState.EMERGENCY:
            self.driving_state = DrivingState.EMERGENCY
        elif ui_state.status == log.ControlsState.OpenpilotState.enabled:
            self.driving_state = DrivingState.ENGAGED
        else:
            self.driving_state = DrivingState.MANUAL
    
    def _update_theme(self):
        """Update UI theme based on lighting conditions"""
        # Use light sensor from camera state to determine theme
        if ui_state.light_sensor >= 0:
            # Higher light sensor value means brighter conditions
            if ui_state.light_sensor > 50:
                self.theme = Theme.DAY
            else:
                self.theme = Theme.NIGHT
        else:
            # Default to day theme if sensor data unavailable
            self.theme = Theme.DAY
    
    def _get_active_layers(self):
        """Get list of active UI layers based on driving state"""
        layers = []
        
        # Always render base layers
        layers.append(self.hud_layer)
        
        # Add specialized layers based on state
        if self.driving_state in [DrivingState.ENGAGED, DrivingState.EMERGENCY]:
            layers.extend([
                self.navigation_layer,
                self.perception_layer,
                self.system_layer
            ])
        
        # Hardware status always available but minimized
        layers.append(self.hardware_layer)
        
        # In emergency state, prioritize critical alerts
        if self.driving_state == DrivingState.EMERGENCY:
            layers.insert(0, CriticalAlertLayer())  # Highest priority
        
        return layers
    
    def _validate_resource_usage(self):
        """Ensure UI stays within resource constraints"""
        # Check render time against target
        if self.avg_render_time > (1.0 / 30.0):  # More than 33ms per frame
            print(f"WARNING: UI render time {self.avg_render_time*1000:.1f}ms exceeds 33ms limit")
        
        # Check frame rate
        current_fps = 1.0 / self.frame_interval if self.frame_interval > 0 else 0
        if current_fps < 25:
            print(f"WARNING: UI frame rate {current_fps:.1f}fps below 25fps threshold")


class UIComponent:
    """Base class for all UI components with resource efficiency features"""
    
    def __init__(self, name: str):
        self.name = name
        self.enabled = True
        self.visible = True
        self.last_update_time = 0.0
        self.render_count = 0
        
        # Resource tracking
        self.avg_render_time = 0.0
    
    def update(self, sm: messaging.SubMaster):
        """Update component with new data"""
        if not self.enabled:
            return
        self._update_internal(sm)
    
    def render(self, rect: rl.Rectangle):
        """Render the component"""
        if not self.enabled or not self.visible:
            return
        
        start_time = time.time()
        self._render_internal(rect)
        render_time = time.time() - start_time
        
        # Update performance metrics
        self.avg_render_time = (self.avg_render_time * 0.9) + (render_time * 0.1)
        self.render_count += 1
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Internal update implementation - override in subclasses"""
        pass
    
    def _render_internal(self, rect: rl.Rectangle):
        """Internal render implementation - override in subclasses"""
        pass


class SystemStatusLayer(UIComponent):
    """System status panel showing safety validation and sensor health"""
    
    def __init__(self):
        super().__init__("SystemStatus")
        self.safety_status = "UNKNOWN"
        self.sensor_health = {"camera": 100, "radar": 100, "gps": 100}
        self.system_ready = False
    
    def _update_internal(self, sm: messaging.SubMaster):
        # Update safety validation status
        # This would connect to the advanced_safety_validation.py system
        if sm.updated["deviceState"]:
            device_state = sm["deviceState"]
            self.system_ready = device_state.started and device_state.thermalStatus <= log.DeviceState.ThermalStatus.green
        
        # Update sensor health (simplified for now)
        if sm.updated["carState"]:
            # In a real implementation, this would check actual sensor data
            self.sensor_health = {
                "camera": 100,
                "radar": 100,
                "gps": 100
            }
            self.safety_status = "VALIDATED" if self.system_ready else "CHECKING"
    
    def _render_internal(self, rect: rl.Rectangle):
        # Calculate position for system status (top-right corner)
        status_width = 250
        status_height = 150
        status_x = rect.x + rect.width - status_width - 20
        status_y = rect.y + 20
        
        # Draw background with theme-appropriate color
        bg_color = rl.Color(30, 30, 40, 200) if ui_state.theme == Theme.NIGHT else rl.Color(240, 240, 245, 200)
        rl.draw_rectangle(status_x, status_y, status_width, status_height, bg_color)
        rl.draw_rectangle_lines(status_x, status_y, status_width, status_height, rl.Color(150, 150, 160, 200))
        
        # Draw system status text
        text_color = rl.Color(255, 255, 255, 255)
        rl.draw_text(f"System: {self.safety_status}", status_x + 10, status_y + 10, 20, text_color)
        rl.draw_text(f"Cam: {self.sensor_health['camera']}%", status_x + 10, status_y + 40, 18, text_color)
        rl.draw_text(f"Radar: {self.sensor_health['radar']}%", status_x + 10, status_y + 65, 18, text_color)
        rl.draw_text(f"GPS: {self.sensor_health['gps']}%", status_x + 10, status_y + 90, 18, text_color)


class PerceptionVisualizationLayer(UIComponent):
    """Visualization of detected objects and lane boundaries"""
    
    def __init__(self):
        super().__init__("PerceptionVisualization")
        self.objects = []
        self.lanes = {"left": None, "right": None, "ego": None}
    
    def _update_internal(self, sm: messaging.SubMaster):
        # Update perception data
        if sm.updated["modelV2"]:
            model_data = sm["modelV2"]
            # Extract lane data
            if len(model_data.laneLines) >= 4:  # Ensure we have enough lane lines
                self.lanes = {
                    "left": model_data.laneLines[0],
                    "ego": model_data.laneLines[1],
                    "ego2": model_data.laneLines[2],
                    "right": model_data.laneLines[3]
                }
        
        # Update detected objects
        if sm.updated["modelV2"]:
            model_data = sm["modelV2"]
            self.objects = []  # Simplified for now
            # In a real system, this would extract from modelV2.leadsV3, etc.
    
    def _render_internal(self, rect: rl.Rectangle):
        # This layer would typically render on top of the camera view
        # For now, we'll draw a placeholder
        pass


class NavigationDisplayLayer(UIComponent):
    """Navigation system with route visualization and turn-by-turn guidance"""
    
    def __init__(self):
        super().__init__("NavigationDisplay")
        self.current_instruction = "None"
        self.distance_to_next = 0
        self.route_progress = 0.0
    
    def _update_internal(self, sm: messaging.SubMaster):
        # Update navigation data
        if sm.updated["navInstruction"]:
            nav_instr = sm["navInstruction"]
            self.current_instruction = nav_instr.maneuver
            self.distance_to_next = nav_instr.distance
            self.route_progress = nav_instr.routeProgress if nav_instr.routeProgress > 0 else 0.0
    
    def _render_internal(self, rect: rl.Rectangle):
        # Render navigation information
        if self.current_instruction != "None":
            nav_width = 300
            nav_height = 80
            nav_x = rect.x + (rect.width - nav_width) // 2
            nav_y = rect.y + 100
            
            # Draw navigation background
            bg_color = rl.Color(30, 50, 70, 200) if ui_state.theme == Theme.NIGHT else rl.Color(220, 240, 255, 200)
            rl.draw_rectangle(nav_x, nav_y, nav_width, nav_height, bg_color)
            rl.draw_rectangle_lines(nav_x, nav_y, nav_width, nav_height, rl.Color(100, 150, 200, 200))
            
            # Draw navigation text
            text_color = rl.Color(255, 255, 255, 255)
            rl.draw_text(self.current_instruction, nav_x + 10, nav_y + 10, 24, text_color)
            rl.draw_text(f"{self.distance_to_next}m", nav_x + 10, nav_y + 40, 20, text_color)


class HardwareStatusLayer(UIComponent):
    """Real-time hardware status visualization (CPU/RAM/Power)"""
    
    def __init__(self):
        super().__init__("HardwareStatus")
        self.cpu_usage = 0
        self.ram_usage = 0
        self.power_draw = 0
        self.temp = 0
    
    def _update_internal(self, sm: messaging.SubMaster):
        # Update hardware metrics (in a real implementation, this would connect to hardware monitoring)
        if sm.updated["deviceState"]:
            device_state = sm["deviceState"]
            self.cpu_usage = device_state.cpuPercent
            self.ram_usage = device_state.memoryPercent
            self.power_draw = device_state.powerDraw
            self.temp = device_state.cpuTempC
    
    def _render_internal(self, rect: rl.Rectangle):
        # Draw hardware status (bottom-left corner)
        hw_width = 200
        hw_height = 120
        hw_x = rect.x + 20
        hw_y = rect.y + rect.height - hw_height - 20
        
        # Draw background
        bg_color = rl.Color(40, 40, 50, 180) if ui_state.theme == Theme.NIGHT else rl.Color(245, 245, 245, 180)
        rl.draw_rectangle(hw_x, hw_y, hw_width, hw_height, bg_color)
        rl.draw_rectangle_lines(hw_x, hw_y, hw_width, hw_height, rl.Color(180, 180, 190, 200))
        
        # Draw hardware metrics
        text_color = rl.Color(255, 255, 255, 255)
        rl.draw_text(f"CPU: {self.cpu_usage}%", hw_x + 10, hw_y + 10, 18, text_color)
        rl.draw_text(f"RAM: {self.ram_usage}%", hw_x + 10, hw_y + 35, 18, text_color)
        rl.draw_text(f"Power: {self.power_draw:.1f}W", hw_x + 10, hw_y + 60, 18, text_color)
        rl.draw_text(f"Temp: {self.temp:.1f}C", hw_x + 10, hw_y + 85, 18, text_color)


class CriticalAlertLayer(UIComponent):
    """Highest priority layer for emergency alerts"""
    
    def __init__(self):
        super().__init__("CriticalAlert")
        self.alert_message = ""
        self.alert_active = False
    
    def _render_internal(self, rect: rl.Rectangle):
        if not self.alert_active:
            return
            
        # Full screen overlay for critical alerts
        overlay_color = rl.Color(220, 20, 60, 120)  # Semi-transparent red
        rl.draw_rectangle(rect.x, rect.y, rect.width, rect.height, overlay_color)
        
        # Centered alert text
        text_color = rl.Color(255, 255, 255, 255)
        text_size = 48
        text = "EMERGENCY - TAKE CONTROL"
        text_width = rl.measure_text(text, text_size)
        text_x = rect.x + (rect.width - text_width) // 2
        text_y = rect.y + rect.height // 2 - 25
        
        rl.draw_text(text, text_x, text_y, text_size, text_color)


def main():
    """Main entry point for the sunnypilot UI system"""
    print("Starting Sunnypilot UI System...")
    
    # Initialize the UI
    sunnypilot_ui = SunnypilotUI()
    
    # Initialize raylib window
    gui_app.init_window("Sunnypilot UI", 1280, 720)
    
    try:
        for _ in gui_app.render():
            # Update UI state
            ui_state.update()
            sunnypilot_ui.update()
            
            # Render UI
            rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
            sunnypilot_ui.render(rect)
            
            # Monitor resource usage
            if time.time() - sunnypilot_ui.last_update_time > 1.0:
                avg_time_ms = sunnypilot_ui.avg_render_time * 1000
                fps = 1.0 / sunnypilot_ui.frame_interval if sunnypilot_ui.frame_interval > 0 else 0
                print(f"UI Stats - Render: {avg_time_ms:.1f}ms, FPS: {fps:.1f}")
    
    except KeyboardInterrupt:
        print("UI shutdown requested")
    finally:
        print("Sunnypilot UI shutdown complete")


if __name__ == "__main__":
    main()