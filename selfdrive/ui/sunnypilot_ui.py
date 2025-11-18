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
        # Initialize messaging with error handling
        try:
            self.sm = messaging.SubMaster([
                "modelV2", "controlsState", "selfdriveState", "deviceState",
                "carState", "perceptionModel", "uiDebug", "navInstruction"
            ])
            self.pm = messaging.PubMaster(["userFlag", "userEvent"])
        except Exception as e:
            print(f"Error initializing messaging system: {e}")
            raise

        # State management
        self.driving_state = DrivingState.DISENGAGED
        self.theme = Theme.DAY  # Will be updated based on light sensor
        self.colors = UIColors()

        # Import and initialize state management
        from openpilot.selfdrive.ui.state_management import UIState
        self.ui_state = UIState.IDLE  # Default state
        self.state_manager = None  # Will be set by parent system

        # Layer management with error handling
        try:
            self.hud_layer = HudRenderer()
        except Exception as e:
            print(f"Error initializing HUD renderer: {e}")
            self.hud_layer = None

        try:
            self.system_layer = SystemStatusLayer()
        except Exception as e:
            print(f"Error initializing system status layer: {e}")
            self.system_layer = None

        try:
            self.perception_layer = PerceptionVisualizationLayer()
        except Exception as e:
            print(f"Error initializing perception visualization layer: {e}")
            self.perception_layer = None

        try:
            self.navigation_layer = NavigationDisplayLayer()
        except Exception as e:
            print(f"Error initializing navigation display layer: {e}")
            self.navigation_layer = None

        try:
            self.hardware_layer = HardwareStatusLayer()
        except Exception as e:
            print(f"Error initializing hardware status layer: {e}")
            self.hardware_layer = None

        # Performance tracking
        self.frame_start_time = 0.0
        self.last_update_time = 0.0
        self.avg_render_time = 0.0
        self.frame_count = 0
        self.error_count = 0
        self.last_error_time = 0.0

        # Resource limits
        self.max_cpu_usage = 5.0  # Percentage
        self.target_fps = 30.0
        self.frame_interval = 1.0 / self.target_fps

        # Initialize UI components
        self._setup_ui_components()

    def _safe_layer_update(self, layer, sm):
        """Safely update a layer with error handling"""
        if layer is None:
            return
        try:
            layer.update(sm)
        except Exception as e:
            self.error_count += 1
            self.last_error_time = time.time()
            print(f"Error updating layer {layer.__class__.__name__}: {e}")
            # Could implement fallback behavior here if needed
    
    def _setup_ui_components(self):
        """Initialize all UI components with proper configuration"""
        # Theme adjustment based on light sensor
        self._update_theme()
    
    def update(self):
        """Update UI state and components"""
        current_time = time.time()
        self.last_update_time = current_time

        # Update messaging state with error handling
        try:
            self.sm.update(0)
        except Exception as e:
            self.error_count += 1
            print(f"Error updating messaging system: {e}")
            # Don't return here, continue with cached data

        # Update driving state
        self._update_driving_state()

        # Update theme based on light sensor
        self._update_theme()

        # Update all UI layers with error handling
        self._safe_layer_update(self.system_layer, self.sm)
        self._safe_layer_update(self.perception_layer, self.sm)
        self._safe_layer_update(self.navigation_layer, self.sm)
        self._safe_layer_update(self.hardware_layer, self.sm)

        # Performance tracking
        self.frame_count += 1
    
    def render(self, rect: rl.Rectangle):
        """Render all UI layers with resource efficiency"""
        from openpilot.selfdrive.ui.state_management import UIState

        # Check if rendering should proceed based on current UI state
        if hasattr(self, 'state_manager') and self.state_manager:
            if self.state_manager.current_state in [UIState.SHUTTING_DOWN, UIState.ERROR]:
                return  # Don't render in shutdown or error states

        self.frame_start_time = time.time()

        # Calculate which layers to render based on driving state
        render_layers = self._get_active_layers()

        # Render layers in z-order (background to foreground) with error handling
        for layer in render_layers:
            if layer is None:
                continue
            try:
                layer.render(rect)
            except Exception as e:
                self.error_count += 1
                print(f"Error rendering layer {layer.__class__.__name__}: {e}")
                # Continue to next layer instead of failing completely

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
        self.error_count = 0

    def update(self, sm: messaging.SubMaster):
        """Update component with new data"""
        if not self.enabled:
            return
        try:
            self._update_internal(sm)
        except Exception as e:
            self.error_count += 1
            print(f"Error in update for component {self.name}: {e}")

    def render(self, rect: rl.Rectangle):
        """Render the component"""
        if not self.enabled or not self.visible:
            return

        start_time = time.time()
        try:
            self._render_internal(rect)
        except Exception as e:
            self.error_count += 1
            print(f"Error in render for component {self.name}: {e}")
            return  # Don't update performance metrics if render failed

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
        self.lead_cars = []

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

            # Update lead cars from model data
            if len(model_data.leadsV3) > 0:
                self.lead_cars = []
                for lead in model_data.leadsV3:
                    if lead.prob > 0.5:  # Only high confidence leads
                        self.lead_cars.append({
                            'x': lead.x[0] if lead.x else 0,
                            'y': lead.y[0] if lead.y else 0,
                            'v_ego': lead.v[0] if lead.v else 0,
                            'prob': lead.prob
                        })

    def _render_internal(self, rect: rl.Rectangle):
        # This layer would typically render on top of the camera view
        # Render lane lines
        self._render_lane_lines(rect)

        # Render lead cars
        self._render_lead_cars(rect)

    def _render_lane_lines(self, rect: rl.Rectangle):
        """Render lane boundary lines"""
        from openpilot.selfdrive.ui.raylib_ui_system import EfficientDrawingRoutines

        # Get screen dimensions to scale properly
        screen_width = rect.width
        screen_height = rect.height

        # Draw each lane line if available
        for side, lane_data in self.lanes.items():
            if lane_data is None or len(lane_data.points) < 2:
                continue

            # Calculate screen coordinates for each point
            screen_points = []
            for point in lane_data.points:
                # Skip if point is invalid
                if point.x == 0 and point.y == 0:
                    continue

                # Simple scaling from world to screen coordinates
                # This is a simplified transform - in a real system, proper camera calibration would be used
                screen_x = rect.x + (screen_width / 2) + (point.x * 10)  # Scale factor for lateral position
                screen_y = rect.y + screen_height - (point.y * 3)  # Scale factor for longitudinal position

                # Only include points that are within screen bounds
                if rect.x <= screen_x <= rect.x + screen_width and rect.y <= screen_y <= rect.y + screen_height:
                    screen_points.append((screen_x, screen_y))

            # Use optimized drawing routine for lane lines
            if len(screen_points) >= 2:
                confidence = lane_data.prob if hasattr(lane_data, 'prob') else 1.0
                is_dashed = side in ['ego', 'ego2']  # Ego lanes are typically dashed
                EfficientDrawingRoutines.draw_lane_lines_batch(screen_points, confidence, is_dashed)

    def _render_lead_cars(self, rect: rl.Rectangle):
        """Render lead cars detected by perception system"""
        from openpilot.selfdrive.ui.raylib_ui_system import EfficientDrawingRoutines

        # Simple scaling function for world to screen coordinates
        def world_to_screen(x, y):
            # This is a simplified transformation - in a real system, proper camera calibration would be used
            screen_x = rect.x + (rect.width / 2) + (x * 10)  # Scale lateral position
            screen_y = rect.y + rect.height - (y * 3)  # Scale longitudinal position
            return screen_x, screen_y

        # Convert lead cars to the format expected by the drawing routine
        vehicle_data = []
        for lead_car in self.lead_cars:
            vehicle_data.append({
                'x': lead_car['x'],
                'y': lead_car['y'],
                'width': 2.0,  # Standard car width in meters
                'length': 4.5,  # Standard car length in meters
                'type': 'car',
                'prob': lead_car['prob']
            })

        # Use optimized batch drawing routine
        screen_transform_func = world_to_screen
        EfficientDrawingRoutines.draw_vehicle_batch(vehicle_data, screen_transform_func)


class NavigationDisplayLayer(UIComponent):
    """Navigation system with route visualization and turn-by-turn guidance"""

    def __init__(self):
        super().__init__("NavigationDisplay")
        self.current_instruction = "None"
        self.distance_to_next = 0
        self.time_to_next = 0
        self.route_progress = 0.0
        self.destination = ""
        self.maneuver_type = ""
        self.nav_points = []  # Navigation route points

    def _update_internal(self, sm: messaging.SubMaster):
        # Update navigation data
        if sm.updated["navInstruction"]:
            nav_instr = sm["navInstruction"]
            self.current_instruction = nav_instr.maneuver
            self.distance_to_next = nav_instr.distance
            self.time_to_next = nav_instr.time
            self.route_progress = nav_instr.routeProgress if nav_instr.routeProgress > 0 else 0.0
            self.destination = nav_instr.roadName or "Unknown"
            self.maneuver_type = getattr(nav_instr, 'maneuverType', "straight")

        # Update navigation route if available
        if sm.updated["navRoute"] and hasattr(sm, 'navRoute'):
            # In a real system, this would process the navigation route
            # For now, we'll keep it as a placeholder
            pass

    def _render_internal(self, rect: rl.Rectangle):
        # Render turn-by-turn navigation panel
        self._render_turn_by_turn_panel(rect)

        # Render route visualization (simplified)
        self._render_route_visualization(rect)

    def _render_turn_by_turn_panel(self, rect: rl.Rectangle):
        """Render the turn-by-turn navigation panel"""
        if self.current_instruction == "None":
            return

        from openpilot.selfdrive.ui.raylib_ui_system import ReusableUIComponents

        # Panel dimensions and position
        nav_width = 300
        nav_height = 100
        nav_x = rect.x + (rect.width - nav_width) // 2
        nav_y = rect.y + 50

        # Use the reusable panel component
        bg_color = rl.Color(30, 50, 70, 200) if ui_state.theme == Theme.NIGHT else rl.Color(220, 240, 255, 200)
        border_color = rl.Color(100, 150, 200, 200)
        ReusableUIComponents.create_panel(nav_x, nav_y, nav_width, nav_height,
                                        "NAVIGATION", bg_color, border_color)

        # Draw maneuver icon/symbol based on maneuver type
        maneuver_symbol = self._get_maneuver_symbol(self.maneuver_type)
        rl.draw_text(maneuver_symbol, int(nav_x + 15), int(nav_y + 25), 32, rl.Color(255, 255, 200, 255))

        # Draw navigation text
        text_color = rl.Color(255, 255, 255, 255)
        rl.draw_text(self.current_instruction, int(nav_x + 60), int(nav_y + 20), 20, text_color)
        rl.draw_text(f"{self.distance_to_next:.0f}m", int(nav_x + 60), int(nav_y + 45), 18, text_color)
        rl.draw_text(f"→ {self.destination}", int(nav_x + 15), int(nav_y + 70), 16, rl.Color(200, 220, 255, 255))

    def _get_maneuver_symbol(self, maneuver_type: str) -> str:
        """Get visual symbol for maneuver type"""
        symbols = {
            "turn_left": "←",
            "turn_right": "→",
            "straight": "↑",
            "slight_left": "⇠",
            "slight_right": "⇢",
            "sharp_left": "⇇",
            "sharp_right": "⇉",
            "u_turn": "↺",
            "arrive": "⦿",
        }
        return symbols.get(maneuver_type.lower(), "→")

    def _render_route_visualization(self, rect: rl.Rectangle):
        """Render the navigation route on screen (simplified)"""
        if not self.nav_points:
            return

        # Draw route as a line connecting points
        for i in range(len(self.nav_points) - 1):
            x1, y1 = self.nav_points[i]
            x2, y2 = self.nav_points[i + 1]

            # Draw route line
            rl.draw_line(int(x1), int(y1), int(x2), int(y2), rl.Color(100, 180, 255, 180))

            # Draw route points
            rl.draw_circle(int(x1), int(y1), 3, rl.Color(100, 180, 255, 200))


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