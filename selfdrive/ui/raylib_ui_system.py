"""
Raylib-based UI Implementation for Sunnypilot
Lightweight, efficient rendering optimized for ARM processors
"""
import pyray as rl
import time
from typing import List, Dict, Callable, Optional
from dataclasses import dataclass
from enum import Enum
import numpy as np

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app, DEFAULT_FPS
from openpilot.selfdrive.ui.sunnypilot_ui import UIComponent, DrivingState
from openpilot.selfdrive.ui.components.hardware_status import HardwareStatusDashboard, CompactHardwareStatus
from openpilot.selfdrive.ui.components.navigation_display import NavigationDisplay, DestinationInputPanel
from openpilot.selfdrive.ui.components.perception_visualization import PerceptionVisualization, PerceptionDebugOverlay
from openpilot.selfdrive.ui.components.system_status import SystemStatusPanel, CompactSystemStatus
from openpilot.selfdrive.ui.components.enhanced_system_status import EnhancedSystemStatusPanel, CompactEnhancedStatus
from openpilot.selfdrive.ui.components.monitoring_visualization import SystemMonitoringDashboard, ValidationMetricsVisualizer
from openpilot.selfdrive.ui.components.controls_interface import ControlsInterface, EmergencyControls, DrivingModeIndicator


class UILayer(Enum):
    """Z-ordering layers for UI elements"""
    BACKGROUND = 0      # Base layer (camera feed, etc.)
    PERCEPTION = 1      # Perception visualization overlay
    NAVIGATION = 2      # Navigation information
    SYSTEM_STATUS = 3   # System health indicators
    CONTROLS = 4        # User controls
    ALERTS = 5          # Alerts and notifications
    DEBUG = 6           # Debug information (topmost)


@dataclass
class UIConfig:
    """Configuration for UI rendering"""
    target_fps: int = 30
    max_cpu_usage: float = 5.0  # Percentage
    resource_budget: float = 50.0  # MB
    screen_width: int = 1280
    screen_height: int = 720
    use_compact_ui: bool = False  # For resource-constrained scenarios
    enable_dynamic_scaling: bool = True  # Enable dynamic UI scaling based on performance
    max_components_per_frame: int = 50  # Limit UI components rendered per frame to maintain performance
    text_cache_size: int = 100  # Maximum number of text size calculations to cache
    enable_texture_caching: bool = True  # Enable texture caching for better performance
    performance_monitoring_interval: float = 1.0  # Interval to check performance metrics (seconds)


class UIStateManager:
    """Manages the state of UI elements and their visibility based on driving state"""
    
    def __init__(self):
        self.driving_state = DrivingState.DISENGAGED
        self.ui_enabled = True
        self.debug_mode = False
        self.compact_mode = False  # Resource-saving mode
        self.settings_visible = False
        self.destination_input_visible = False
        self.last_update_time = 0.0
        self.resource_warning = False
    
    def update_state(self, driving_state: DrivingState, resource_usage: Dict[str, float]):
        """Update UI state based on driving state and resource usage"""
        self.driving_state = driving_state
        
        # Check resource usage and enable compact mode if needed
        cpu_usage = resource_usage.get('cpu', 0)
        ram_usage = resource_usage.get('ram', 0)
        
        self.resource_warning = cpu_usage > 80.0 or ram_usage > 85.0
        self.compact_mode = self.resource_warning or (cpu_usage > 50.0 and ram_usage > 70.0)
        
        # Determine UI visibility based on driving state
        self.ui_enabled = True  # Always enabled for now
    
    def get_component_visibility(self, component_name: str) -> bool:
        """Determine if a component should be visible based on current state"""
        # Always show critical components
        if component_name in ['DrivingModeIndicator', 'EmergencyControls']:
            return True
        
        # Hide some components in manual driving mode
        if self.driving_state == DrivingState.MANUAL and component_name in [
            'NavigationDisplay', 'PerceptionVisualization', 'SystemStatusPanel'
        ]:
            return False
        
        # In emergency mode, only show critical components
        if self.driving_state == DrivingState.EMERGENCY:
            return component_name in [
                'EmergencyControls', 'DrivingModeIndicator', 'CompactSystemStatus'
            ]
        
        return True


class RaylibUI:
    """Main UI system using raylib for rendering with advanced performance and resource management"""

    def __init__(self, config: UIConfig = None):
        self.config = config or UIConfig()

        # Messaging
        self.sm = messaging.SubMaster([
            "modelV2", "controlsState", "selfdriveState", "deviceState",
            "carState", "perceptionModel", "uiDebug", "navInstruction", "navRoute"
        ])
        self.pm = messaging.PubMaster(["userFlag", "userEvent"])

        # UI State Management
        self.state_manager = UIStateManager()

        # Initialize all UI components
        self.components: Dict[UILayer, List[UIComponent]] = {
            UILayer.BACKGROUND: [],
            UILayer.PERCEPTION: [],
            UILayer.NAVIGATION: [],
            UILayer.SYSTEM_STATUS: [],
            UILayer.CONTROLS: [],
            UILayer.ALERTS: [],
            UILayer.DEBUG: []
        }

        self._initialize_components()

        # Performance tracking
        self.frame_times = []
        self.max_frame_times = 100  # Keep last 100 frame times
        self.last_frame_time = 0.0
        self.avg_frame_time = 0.0
        self.last_performance_check = time.time()

        # Resource tracking
        self.last_resource_check = 0.0
        self.current_cpu_usage = 0.0
        self.current_ram_usage = 0.0

        # Resource management
        self.component_render_counter = 0
        self.frame_component_limit = self.config.max_components_per_frame

        # Dynamic scaling based on resource usage
        self.dynamic_scale_factor = 1.0
        self.performance_history = []
        self.max_performance_history = 50  # Keep last 50 performance samples

        # Callbacks for control actions
        self.engage_callback: Optional[Callable] = None
        self.disengage_callback: Optional[Callable] = None
        self.settings_callback: Optional[Callable] = None
    
    def _initialize_components(self):
        """Initialize all UI components and assign them to appropriate layers"""
        # Perception layer
        perception_viz = PerceptionVisualization()
        perception_debug = PerceptionDebugOverlay()
        self.components[UILayer.PERCEPTION] = [perception_viz, perception_debug]
        
        # Navigation layer
        nav_display = NavigationDisplay()
        dest_input = DestinationInputPanel()
        self.components[UILayer.NAVIGATION] = [nav_display]
        # Destination input is a special case, handled separately
        self.destination_input = dest_input
        
        # System status layer
        if self.config.use_compact_ui:
            system_status = CompactEnhancedStatus()
        else:
            system_status = EnhancedSystemStatusPanel()
        self.components[UILayer.SYSTEM_STATUS] = [system_status]
        
        # System monitoring dashboard (in system status layer)
        monitoring_dashboard = SystemMonitoringDashboard()
        validation_visualizer = ValidationMetricsVisualizer(580, 160, 200, 150)  # Positioned below the dashboard
        system_status = self.components[UILayer.SYSTEM_STATUS][0]  # Get the existing system status panel

        # Replace with list containing both system status and monitoring
        self.components[UILayer.SYSTEM_STATUS] = [system_status, monitoring_dashboard, validation_visualizer]

        # Controls layer
        controls = ControlsInterface()
        mode_indicator = DrivingModeIndicator()
        self.components[UILayer.CONTROLS] = [controls, mode_indicator]
        
        # Debug layer
        if self.state_manager.debug_mode:
            debug_comp = UIComponent("DebugInfo")
            self.components[UILayer.DEBUG] = [debug_comp]
        
        # Emergency controls (special case that gets rendered when needed)
        self.emergency_controls = EmergencyControls()
    
    def set_control_callbacks(self, engage_callback: Callable = None,
                             disengage_callback: Callable = None,
                             settings_callback: Callable = None):
        """Set callbacks for control actions"""
        self.engage_callback = engage_callback
        self.disengage_callback = disengage_callback
        self.settings_callback = settings_callback
        
        # Set callbacks on controls component
        controls = self._get_component("ControlsInterface")
        if controls:
            controls.set_callbacks(
                engage_callback=engage_callback,
                disengage_callback=disengage_callback,
                settings_callback=settings_callback
            )
    
    def _get_component(self, name: str) -> Optional[UIComponent]:
        """Get a component by name"""
        for layer_components in self.components.values():
            for component in layer_components:
                if component.name == name:
                    return component
        return None
    
    def update(self):
        """Update all UI components with new data"""
        current_time = time.time()
        
        # Update messaging
        self.sm.update(0)
        
        # Update resource usage
        if self.sm.updated["deviceState"]:
            device_state = self.sm["deviceState"]
            self.current_cpu_usage = device_state.cpuPercent
            self.current_ram_usage = device_state.memoryPercent
        
        # Update resource tracking
        resource_usage = {
            'cpu': self.current_cpu_usage,
            'ram': self.current_ram_usage
        }
        
        # Get current driving state
        driving_state = DrivingState.DISENGAGED
        if self.sm.updated["selfdriveState"]:
            selfdrive_state = self.sm["selfdriveState"]
            if selfdrive_state.state == 4:  # Emergency
                driving_state = DrivingState.EMERGENCY
            elif selfdrive_state.enabled:
                driving_state = DrivingState.ENGAGED
            else:
                driving_state = DrivingState.DISENGAGED
        
        # Update UI state
        self.state_manager.update_state(driving_state, resource_usage)
        
        # Update emergency controls separately
        self.emergency_controls.update(self.sm)
        
        # Update destination input panel separately
        self.destination_input.update(self.sm)
        
        # Update all components in all layers
        for layer in UILayer:
            for component in self.components[layer]:
                # Set visibility based on context
                component.visible = self.state_manager.get_component_visibility(component.name)
                
                # Update component if it's enabled and visible
                if component.enabled and component.visible:
                    component.update(self.sm)
    
    def render(self, rect: rl.Rectangle):
        """Render all UI layers with proper z-ordering and resource management"""
        start_time = time.time()

        # Check if we need to adjust UI scaling based on performance
        if (time.time() - self.last_performance_check) > self.config.performance_monitoring_interval:
            self._adjust_dynamic_scaling()
            self.last_performance_check = time.time()

        # Reset component render counter for this frame
        self.component_render_counter = 0

        # Render layers in order (background to foreground)
        for layer in UILayer:
            if self.component_render_counter >= self.frame_component_limit:
                break  # Stop rendering if we've hit the component limit for this frame

            for component in self.components[layer]:
                if self.component_render_counter >= self.frame_component_limit:
                    break  # Stop if we've hit the limit

                if component.visible and component.enabled:
                    component.render(rect)
                    self.component_render_counter += 1

        # Render emergency controls if active
        if self.emergency_controls.visible and self.component_render_counter < self.frame_component_limit:
            self.emergency_controls.render(rect)
            self.component_render_counter += 1

        # Calculate and store frame time
        self.last_frame_time = time.time() - start_time
        self.frame_times.append(self.last_frame_time)

        # Keep only the last N frame times
        if len(self.frame_times) > self.max_frame_times:
            self.frame_times.pop(0)

        # Calculate average frame time
        if self.frame_times:
            self.avg_frame_time = sum(self.frame_times) / len(self.frame_times)

    def _adjust_dynamic_scaling(self):
        """Adjust UI scaling based on performance to maintain target frame rate"""
        if not self.frame_times or not self.config.enable_dynamic_scaling:
            return

        # Calculate current FPS
        if self.avg_frame_time > 0:
            current_fps = 1.0 / self.avg_frame_time
            target_fps = self.config.target_fps

            # If we're below target FPS, reduce scale factor to improve performance
            if current_fps < target_fps * 0.8:  # 80% of target
                self.dynamic_scale_factor = max(0.7, self.dynamic_scale_factor - 0.05)  # Reduce by 5%
            elif current_fps > target_fps * 1.1:  # 110% of target
                self.dynamic_scale_factor = min(1.0, self.dynamic_scale_factor + 0.02)  # Increase by 2%

            # Apply the new scale factor to the UI system
            ReusableUIComponents.set_scale_factor(self.dynamic_scale_factor)

        # Record performance in history
        self.performance_history.append({
            'timestamp': time.time(),
            'fps': 1.0 / self.avg_frame_time if self.avg_frame_time > 0 else 0,
            'frame_time': self.avg_frame_time,
            'scale_factor': self.dynamic_scale_factor
        })

        # Keep only the last N performance samples
        if len(self.performance_history) > self.max_performance_history:
            self.performance_history.pop(0)
    
    def get_performance_metrics(self) -> Dict[str, float]:
        """Get performance metrics with additional information"""
        current_fps = 1.0 / self.avg_frame_time if self.avg_frame_time > 0 else 0
        render_time_ms = self.avg_frame_time * 1000

        return {
            'fps': current_fps,
            'render_time_ms': render_time_ms,
            'cpu_usage': self.current_cpu_usage,
            'ram_usage': self.current_ram_usage,
            'resource_warning': self.state_manager.resource_warning,
            'scale_factor': self.dynamic_scale_factor,
            'components_rendered': self.component_render_counter,
            'frame_component_limit': self.frame_component_limit,
            'target_fps': self.config.target_fps,
            'performance_history_count': len(self.performance_history)
        }
    
    def enable_debug_mode(self, enable: bool):
        """Enable or disable debug mode"""
        self.state_manager.debug_mode = enable
        if enable and not self.components[UILayer.DEBUG]:
            debug_info = UIComponent("DebugInfo")
            self.components[UILayer.DEBUG] = [debug_info]
        elif not enable:
            self.components[UILayer.DEBUG] = []
    
    def toggle_compact_mode(self):
        """Toggle compact UI mode for better performance"""
        self.state_manager.compact_mode = not self.state_manager.compact_mode


class ReusableUIComponents:
    """Collection of reusable UI components for efficient development with proper resource management"""

    # Class-level resource cache to reuse expensive operations
    _text_cache = {}
    _font_cache = {}
    _scale_factor = 1.0  # For UI scaling

    @classmethod
    def set_scale_factor(cls, scale: float):
        """Set global UI scaling factor"""
        cls._scale_factor = max(0.5, min(2.0, scale))  # Clamp between 0.5 and 2.0

    @classmethod
    def scaled_value(cls, value: float) -> int:
        """Apply scale factor to UI values"""
        return int(value * cls._scale_factor)

    @classmethod
    def cached_text_size(cls, text: str, font_size: int) -> int:
        """Cache text size calculations to avoid repeated expensive calls"""
        key = f"{text}_{font_size}"
        if key not in cls._text_cache:
            cls._text_cache[key] = rl.measure_text(text, cls.scaled_value(font_size))
        return cls._text_cache[key]

    @staticmethod
    def create_status_indicator(x: float, y: float, width: float, height: float,
                              text: str, status: bool,
                              on_color: rl.Color = rl.Color(50, 205, 50, 255),
                              off_color: rl.Color = rl.Color(220, 20, 60, 255)) -> None:
        """Create a generic status indicator with proper scaling"""
        scaled_x = ReusableUIComponents.scaled_value(x)
        scaled_y = ReusableUIComponents.scaled_value(y)
        scaled_width = ReusableUIComponents.scaled_value(width)
        scaled_height = ReusableUIComponents.scaled_value(height)

        # Draw background
        color = on_color if status else off_color
        rl.draw_rectangle(scaled_x, scaled_y, scaled_width, scaled_height,
                         rl.Color(color.r, color.g, color.b, 180))
        rl.draw_rectangle_lines(scaled_x, scaled_y, scaled_width, scaled_height,
                               rl.Color(200, 200, 220, 220))

        # Draw text with proper sizing
        scaled_text_size = ReusableUIComponents.scaled_value(16)
        text_size = ReusableUIComponents.cached_text_size(text, 16)
        rl.draw_text(text, scaled_x + ReusableUIComponents.scaled_value(10),
                    scaled_y + (scaled_height - ReusableUIComponents.scaled_value(20)) // 2,
                    scaled_text_size, rl.WHITE)

    @staticmethod
    def create_progress_bar(x: float, y: float, width: float, height: float,
                           value: float, max_value: float,
                           color: rl.Color = rl.Color(70, 130, 180, 255),
                           bg_color: rl.Color = rl.Color(50, 50, 60, 200),
                           show_text: bool = True) -> None:
        """Create a progress bar component with optional text"""
        scaled_x = ReusableUIComponents.scaled_value(x)
        scaled_y = ReusableUIComponents.scaled_value(y)
        scaled_width = ReusableUIComponents.scaled_value(width)
        scaled_height = ReusableUIComponents.scaled_value(height)

        # Draw background
        rl.draw_rectangle(scaled_x, scaled_y, scaled_width, scaled_height, bg_color)

        # Draw progress
        if max_value > 0:
            progress_width = min(scaled_width, ReusableUIComponents.scaled_value((value / max_value) * width))
            rl.draw_rectangle(scaled_x, scaled_y, progress_width, scaled_height, color)

        # Draw border
        rl.draw_rectangle_lines(scaled_x, scaled_y, scaled_width, scaled_height,
                               rl.Color(150, 150, 160, 220))

        # Optionally show text
        if show_text and max_value > 0:
            percent = int((value / max_value) * 100)
            text = f"{percent}%"
            scaled_text_size = ReusableUIComponents.scaled_value(14)
            text_width = ReusableUIComponents.cached_text_size(text, 14)
            text_x = scaled_x + (scaled_width - text_width) // 2
            text_y = scaled_y + (scaled_height - scaled_text_size) // 2
            rl.draw_text(text, text_x, text_y, scaled_text_size, rl.WHITE)

    @staticmethod
    def create_gauge(x: float, y: float, radius: float, value: float, max_value: float,
                     label: str, color: rl.Color = rl.Color(70, 130, 180, 255)) -> None:
        """Create a circular gauge component with optimization"""
        scaled_x = ReusableUIComponents.scaled_value(x)
        scaled_y = ReusableUIComponents.scaled_value(y)
        scaled_radius = ReusableUIComponents.scaled_value(radius)

        # Draw background circle
        rl.draw_circle_lines(scaled_x, scaled_y, scaled_radius, rl.Color(100, 100, 120, 200))

        # Draw filled arc based on value
        fill_percent = min(1.0, value / max_value) if max_value > 0 else 0
        # Calculate angle for filled portion (0 to 360 degrees)
        angle = fill_percent * 360.0

        # Draw the filled arc - OPTIMIZED: reduce number of lines drawn
        num_segments = max(10, int(angle * 2))  # Adjust resolution based on angle
        for i in range(num_segments):
            # Convert polar to Cartesian coordinates
            segment_angle = (i * angle / num_segments) * 3.14159 / 180 - (3.14159 / 2)  # Start from top
            inner_radius = scaled_radius * 0.8
            outer_radius = scaled_radius

            start_x = scaled_x + inner_radius * np.cos(segment_angle)
            start_y = scaled_y + inner_radius * np.sin(segment_angle)
            end_x = scaled_x + outer_radius * np.cos(segment_angle)
            end_y = scaled_y + outer_radius * np.sin(segment_angle)

            rl.draw_line(int(start_x), int(start_y), int(end_x), int(end_y), color)

        # Draw text in center
        value_text = f"{int(value)}/{int(max_value)}"
        scaled_text_size = ReusableUIComponents.scaled_value(16)
        text_width = ReusableUIComponents.cached_text_size(value_text, 16)
        rl.draw_text(value_text, int(scaled_x - text_width/2), int(scaled_y - ReusableUIComponents.scaled_value(8)),
                    scaled_text_size, rl.WHITE)

        label_text_size = ReusableUIComponents.scaled_value(12)
        label_width = ReusableUIComponents.cached_text_size(label, 12)
        rl.draw_text(label, int(scaled_x - label_width/2), int(scaled_y + ReusableUIComponents.scaled_value(8)),
                    label_text_size, rl.LIGHTGRAY)

    @staticmethod
    def create_panel(x: float, y: float, width: float, height: float,
                    title: str = "", bg_color: rl.Color = None,
                    border_color: rl.Color = None) -> None:
        """Create a reusable panel component with proper scaling"""
        if bg_color is None:
            bg_color = rl.Color(30, 30, 40, 220)  # Default dark panel
        if border_color is None:
            border_color = rl.Color(100, 100, 120, 255)  # Default border

        scaled_x = ReusableUIComponents.scaled_value(x)
        scaled_y = ReusableUIComponents.scaled_value(y)
        scaled_width = ReusableUIComponents.scaled_value(width)
        scaled_height = ReusableUIComponents.scaled_value(height)

        # Draw panel background
        rl.draw_rectangle(scaled_x, scaled_y, scaled_width, scaled_height, bg_color)
        rl.draw_rectangle_lines(scaled_x, scaled_y, scaled_width, scaled_height, border_color)

        # Draw title if provided
        if title:
            scaled_text_size = ReusableUIComponents.scaled_value(16)
            rl.draw_text(title, scaled_x + ReusableUIComponents.scaled_value(10),
                        scaled_y + ReusableUIComponents.scaled_value(5),
                        scaled_text_size, rl.LIGHTGRAY)

    @staticmethod
    def create_button(x: float, y: float, width: float, height: float,
                     text: str, enabled: bool = True) -> bool:
        """Create a button component and return if clicked with proper scaling"""
        scaled_x = ReusableUIComponents.scaled_value(x)
        scaled_y = ReusableUIComponents.scaled_value(y)
        scaled_width = ReusableUIComponents.scaled_value(width)
        scaled_height = ReusableUIComponents.scaled_value(height)

        # Determine colors based on enabled state
        bg_color = rl.Color(70, 130, 180, 255) if enabled else rl.Color(80, 80, 90, 200)
        border_color = rl.Color(200, 200, 220, 255) if enabled else rl.Color(100, 100, 120, 200)
        text_color = rl.WHITE if enabled else rl.Color(150, 150, 160, 255)

        # Draw button background
        rl.draw_rectangle(scaled_x, scaled_y, scaled_width, scaled_height, bg_color)
        rl.draw_rectangle_lines(scaled_x, scaled_y, scaled_width, scaled_height, border_color)

        # Draw centered text
        scaled_text_size = ReusableUIComponents.scaled_value(16)
        text_width = ReusableUIComponents.cached_text_size(text, 16)
        text_x = scaled_x + (scaled_width - text_width) // 2
        text_y = scaled_y + (scaled_height - scaled_text_size) // 2
        rl.draw_text(text, text_x, text_y, scaled_text_size, text_color)

        # Check for click
        mouse_x, mouse_y = rl.get_mouse_x(), rl.get_mouse_y()
        button_rect = rl.Rectangle(scaled_x, scaled_y, scaled_width, scaled_height)

        clicked = (rl.is_mouse_button_pressed(rl.MouseButton.MOUSE_LEFT_BUTTON) and
                   rl.check_collision_point_rec(rl.Vector2(mouse_x, mouse_y), button_rect))

        return clicked if enabled else False


class EfficientDrawingRoutines:
    """Optimized drawing routines for ARM processors with resource management"""

    # Static cache for frequently used resources
    _shape_cache = {}
    _max_cache_size = 100  # Limit cache size to prevent memory issues

    @staticmethod
    def draw_text_cached(text: str, x: float, y: float, font_size: int, color: rl.Color,
                        font: rl.Font = None) -> None:
        """Efficient text drawing with caching"""
        # Use the ReusableUIComponents cache for text sizing
        scaled_x = ReusableUIComponents.scaled_value(x) if hasattr(ReusableUIComponents, '_scale_factor') else int(x)
        scaled_y = ReusableUIComponents.scaled_value(y) if hasattr(ReusableUIComponents, '_scale_factor') else int(y)
        scaled_font_size = ReusableUIComponents.scaled_value(font_size) if hasattr(ReusableUIComponents, '_scale_factor') else font_size

        if font:
            rl.draw_text_ex(font, text, rl.Vector2(scaled_x, scaled_y), scaled_font_size, 0, color)
        else:
            rl.draw_text(text, scaled_x, scaled_y, scaled_font_size, color)

    @staticmethod
    def draw_multiple_rectangles(rectangles: List[tuple], color: rl.Color) -> None:
        """Efficiently draw multiple rectangles using object pooling and reduced calls"""
        # Apply scaling to all rectangles in batch
        scaled_rectangles = [
            (ReusableUIComponents.scaled_value(x),
             ReusableUIComponents.scaled_value(y),
             ReusableUIComponents.scaled_value(w),
             ReusableUIComponents.scaled_value(h))
            for x, y, w, h in rectangles
        ]

        # Draw all rectangles
        for x, y, width, height in scaled_rectangles:
            rl.draw_rectangle(int(x), int(y), int(width), int(height), color)

    @staticmethod
    def draw_lines_batch(lines: List[tuple], color: rl.Color) -> None:
        """Efficiently draw multiple lines with optimization"""
        # Apply scaling to all lines in batch
        scaled_lines = [
            (ReusableUIComponents.scaled_value(x1),
             ReusableUIComponents.scaled_value(y1),
             ReusableUIComponents.scaled_value(x2),
             ReusableUIComponents.scaled_value(y2))
            for x1, y1, x2, y2 in lines
        ]

        # Draw all lines
        for x1, y1, x2, y2 in scaled_lines:
            rl.draw_line(int(x1), int(y1), int(x2), int(y2), color)

    @staticmethod
    def draw_circle_batch(circles: List[tuple], color: rl.Color) -> None:
        """Efficiently draw multiple circles with optimization"""
        # Apply scaling to all circles in batch
        for x, y, radius in circles:
            scaled_x = ReusableUIComponents.scaled_value(x)
            scaled_y = ReusableUIComponents.scaled_value(y)
            scaled_radius = ReusableUIComponents.scaled_value(radius)
            rl.draw_circle(int(scaled_x), int(scaled_y), int(scaled_radius), color)

    @staticmethod
    def draw_lane_lines_batch(lane_points: List[List[tuple]], confidence: float, is_dashed: bool = False) -> None:
        """Optimized drawing for lane lines with proper confidence-based coloring and resource management"""
        if not lane_points or len(lane_points) < 2:
            return

        # Determine color based on confidence
        if confidence > 0.9:
            color = rl.Color(255, 255, 255, int(200 * confidence))  # Bright white for high confidence
        elif confidence > 0.7:
            color = rl.Color(200, 200, 230, int(180 * confidence))  # Slightly dim for medium confidence
        else:
            color = rl.Color(150, 150, 180, int(150 * confidence))  # Dim for low confidence

        # Apply scaling to points
        scaled_points = [
            (ReusableUIComponents.scaled_value(x), ReusableUIComponents.scaled_value(y))
            for x, y in lane_points
        ]

        # Draw lane lines with dashed pattern if needed
        step = 1 if not is_dashed else 2  # For dashed lines, only draw every other segment
        skip = 0 if not is_dashed else 1  # Skip every other segment for dashed effect

        for i in range(len(scaled_points) - 1):
            if is_dashed and i % 4 >= 2:  # Draw every other segment for dashed effect
                continue

            rl.draw_line(int(scaled_points[i][0]), int(scaled_points[i][1]),
                        int(scaled_points[i + 1][0]), int(scaled_points[i + 1][1]), color)

    @staticmethod
    def draw_vehicle_batch(vehicles: List[Dict], screen_transform_func) -> None:
        """Optimized drawing for multiple vehicles with proper size/position transformation and LOD"""
        # Apply level-of-detail based on number of vehicles to maintain performance
        max_vehicles_to_draw = 20  # Limit to prevent performance degradation
        vehicles_to_draw = vehicles[:max_vehicles_to_draw] if len(vehicles) > max_vehicles_to_draw else vehicles

        for vehicle in vehicles_to_draw:
            # Apply screen transformation
            raw_x, raw_y = screen_transform_func(vehicle['x'], vehicle['y'])

            # Apply UI scaling
            screen_x = ReusableUIComponents.scaled_value(raw_x)
            screen_y = ReusableUIComponents.scaled_value(raw_y)

            # Scale dimensions based on UI factor
            base_width = vehicle['width'] * 3  # Approximate scaling
            base_height = vehicle['length'] * 3  # Approximate scaling
            screen_width = ReusableUIComponents.scaled_value(base_width)
            screen_height = ReusableUIComponents.scaled_value(base_height)

            # Determine color based on type and confidence
            if vehicle.get('type', 'car') == 'car':
                base_color = rl.Color(255, 100, 100, 200)  # Red for cars
            elif vehicle['type'] == 'truck':
                base_color = rl.Color(200, 100, 255, 200)  # Purple for trucks
            else:
                base_color = rl.Color(100, 200, 255, 200)  # Blue for others

            # Adjust transparency based on confidence
            alpha = int(min(255, 100 + vehicle.get('prob', 0.5) * 155))
            color = rl.Color(base_color.r, base_color.g, base_color.b, alpha)

            # Draw vehicle as a rectangle
            rl.draw_rectangle(int(screen_x - screen_width//2),
                             int(screen_y - screen_height//2),
                             int(screen_width),
                             int(screen_height),
                             color)
            rl.draw_rectangle_lines(int(screen_x - screen_width//2),
                                  int(screen_y - screen_height//2),
                                  int(screen_width),
                                  int(screen_height),
                                  rl.Color(255, 255, 255, 255))

    @staticmethod
    def clear_cache():
        """Clear drawing cache to free up memory"""
        EfficientDrawingRoutines._shape_cache.clear()


def main():
    """Main entry point for the raylib UI implementation with resource management"""
    print("Initializing Raylib UI for Sunnypilot...")

    # Initialize UI configuration
    config = UIConfig()

    # Initialize raylib window with proper settings for ARM processors
    rl.set_config_flags(rl.ConfigFlags.FLAG_VSYNC_HINT)  # Enable VSYNC for smoother rendering
    rl.init_window(config.screen_width, config.screen_height, "Sunnypilot UI")
    rl.set_target_fps(config.target_fps)

    # Set up UI scaling based on screen resolution
    screen_width, screen_height = rl.get_screen_width(), rl.get_screen_height()
    scale_factor = min(screen_width / 1280.0, screen_height / 720.0)  # Base scaling on 720p
    ReusableUIComponents.set_scale_factor(scale_factor)

    # Initialize UI system
    ui_system = RaylibUI(config)

    print("UI system initialized successfully!")
    print(f"Target FPS: {config.target_fps}")
    print(f"Screen Resolution: {screen_width}x{screen_height}")
    print(f"UI Scale Factor: {scale_factor:.2f}")
    print("Press ESC to exit, D for debug mode, C for compact mode, +/- for UI scaling")

    # Performance tracking
    frame_count = 0
    last_cache_clear_time = time.time()

    try:
        while not rl.window_should_close():
            current_time = time.time()

            # Periodically clear drawing cache to prevent memory issues
            if current_time - last_cache_clear_time > 5.0:  # Clear every 5 seconds
                EfficientDrawingRoutines.clear_cache()
                last_cache_clear_time = current_time

            # Update UI system
            ui_system.update()

            # Begin drawing
            rl.begin_drawing()
            rl.clear_background(rl.BLACK)

            # Render UI
            screen_rect = rl.Rectangle(0, 0, screen_width, screen_height)
            ui_system.render(screen_rect)

            # Handle input
            if rl.is_key_pressed(rl.KeyboardKey.KEY_ESCAPE):
                break
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_D):
                ui_system.enable_debug_mode(not ui_system.state_manager.debug_mode)
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_C):
                ui_system.toggle_compact_mode()
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_EQUAL) or rl.is_key_pressed(rl.KeyboardKey.KEY_KP_ADD):
                # Increase UI scale
                current_scale = ReusableUIComponents._scale_factor
                ReusableUIComponents.set_scale_factor(current_scale + 0.1)
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_MINUS) or rl.is_key_pressed(rl.KeyboardKey.KEY_KP_SUBTRACT):
                # Decrease UI scale
                current_scale = ReusableUIComponents._scale_factor
                ReusableUIComponents.set_scale_factor(current_scale - 0.1)

            # Display performance metrics
            metrics = ui_system.get_performance_metrics()
            perf_text = f"FPS: {metrics['fps']:.1f} | Render: {metrics['render_time_ms']:.1f}ms | Scale: {metrics['scale_factor']:.2f}"
            scaled_text_size = ReusableUIComponents.scaled_value(16)
            rl.draw_text(perf_text, ReusableUIComponents.scaled_value(10),
                        ReusableUIComponents.scaled_value(10),
                        scaled_text_size, rl.WHITE)

            if metrics['resource_warning']:
                warning_text = "RESOURCE WARNING!"
                rl.draw_text(warning_text, ReusableUIComponents.scaled_value(10),
                           ReusableUIComponents.scaled_value(30),
                           scaled_text_size, rl.RED)

            # Update frame count for performance tracking
            frame_count += 1

            # End drawing
            rl.end_drawing()

    except KeyboardInterrupt:
        print("UI shutdown requested")
    finally:
        # Cleanup
        EfficientDrawingRoutines.clear_cache()  # Final cleanup
        rl.close_window()
        print("Raylib UI shutdown complete")


if __name__ == "__main__":
    main()