"""
Raylib-based UI Implementation for Sunnypilot
Lightweight, efficient rendering optimized for ARM processors
"""
import pyray as rl
import time
from typing import List, Dict, Callable, Optional
from dataclasses import dataclass
from enum import Enum

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app, DEFAULT_FPS
from openpilot.selfdrive.ui.sunnypilot_ui import UIComponent, DrivingState
from openpilot.selfdrive.ui.components.hardware_status import HardwareStatusDashboard, CompactHardwareStatus
from openpilot.selfdrive.ui.components.navigation_display import NavigationDisplay, DestinationInputPanel
from openpilot.selfdrive.ui.components.perception_visualization import PerceptionVisualization, PerceptionDebugOverlay
from openpilot.selfdrive.ui.components.system_status import SystemStatusPanel, CompactSystemStatus
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
    """Main UI system using raylib for rendering"""
    
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
        
        # Resource tracking
        self.last_resource_check = 0.0
        self.current_cpu_usage = 0.0
        self.current_ram_usage = 0.0
        
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
            system_status = CompactSystemStatus()
        else:
            system_status = SystemStatusPanel()
        self.components[UILayer.SYSTEM_STATUS] = [system_status]
        
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
        """Render all UI layers with proper z-ordering"""
        start_time = time.time()
        
        # Render layers in order (background to foreground)
        for layer in UILayer:
            for component in self.components[layer]:
                if component.visible and component.enabled:
                    component.render(rect)
        
        # Render emergency controls if active
        if self.emergency_controls.visible:
            self.emergency_controls.render(rect)
        
        # Calculate and store frame time
        self.last_frame_time = time.time() - start_time
        self.frame_times.append(self.last_frame_time)
        
        # Keep only the last N frame times
        if len(self.frame_times) > self.max_frame_times:
            self.frame_times.pop(0)
        
        # Calculate average frame time
        if self.frame_times:
            self.avg_frame_time = sum(self.frame_times) / len(self.frame_times)
    
    def get_performance_metrics(self) -> Dict[str, float]:
        """Get performance metrics"""
        current_fps = 1.0 / self.avg_frame_time if self.avg_frame_time > 0 else 0
        render_time_ms = self.avg_frame_time * 1000
        
        return {
            'fps': current_fps,
            'render_time_ms': render_time_ms,
            'cpu_usage': self.current_cpu_usage,
            'ram_usage': self.current_ram_usage,
            'resource_warning': self.state_manager.resource_warning
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
    """Collection of reusable UI components for efficient development"""
    
    @staticmethod
    def create_status_indicator(x: float, y: float, width: float, height: float,
                              text: str, status: bool, 
                              on_color: rl.Color = rl.Color(50, 205, 50, 255),
                              off_color: rl.Color = rl.Color(220, 20, 60, 255)) -> None:
        """Create a generic status indicator"""
        # Draw background
        color = on_color if status else off_color
        rl.draw_rectangle(int(x), int(y), int(width), int(height), 
                         rl.Color(color.r, color.g, color.b, 180))
        rl.draw_rectangle_lines(int(x), int(y), int(width), int(height), 
                               rl.Color(200, 200, 220, 220))
        
        # Draw text
        rl.draw_text(text, int(x + 10), int(y + (height - 20) / 2), 16, rl.WHITE)
    
    @staticmethod
    def create_progress_bar(x: float, y: float, width: float, height: float,
                           value: float, max_value: float,
                           color: rl.Color = rl.Color(70, 130, 180, 255),
                           bg_color: rl.Color = rl.Color(50, 50, 60, 200)) -> None:
        """Create a progress bar component"""
        # Draw background
        rl.draw_rectangle(int(x), int(y), int(width), int(height), bg_color)
        
        # Draw progress
        if max_value > 0:
            progress_width = (value / max_value) * width
            rl.draw_rectangle(int(x), int(y), int(progress_width), int(height), color)
        
        # Draw border
        rl.draw_rectangle_lines(int(x), int(y), int(width), int(height), 
                               rl.Color(150, 150, 160, 220))
    
    @staticmethod
    def create_gauge(x: float, y: float, radius: float, value: float, max_value: float,
                     label: str, color: rl.Color = rl.Color(70, 130, 180, 255)) -> None:
        """Create a circular gauge component"""
        # Draw background circle
        rl.draw_circle_lines(int(x), int(y), int(radius), rl.Color(100, 100, 120, 200))
        
        # Draw filled arc based on value
        fill_percent = min(1.0, value / max_value) if max_value > 0 else 0
        angle = fill_percent * 270  # 270 degrees (quarter circle)
        
        # Draw text in center
        rl.draw_text(f"{int(value)}/{int(max_value)}", int(x - 30), int(y - 10), 16, rl.WHITE)
        rl.draw_text(label, int(x - 20), int(y + 10), 12, rl.LIGHTGRAY)


class EfficientDrawingRoutines:
    """Optimized drawing routines for ARM processors"""
    
    @staticmethod
    def draw_text_cached(text: str, x: float, y: float, font_size: int, color: rl.Color,
                        font: rl.Font = None) -> None:
        """Efficient text drawing with caching"""
        if font:
            rl.draw_text_ex(font, text, rl.Vector2(x, y), font_size, 0, color)
        else:
            rl.draw_text(text, int(x), int(y), font_size, color)
    
    @staticmethod
    def draw_multiple_rectangles(rectangles: List[tuple], color: rl.Color) -> None:
        """Efficiently draw multiple rectangles in one call"""
        for rect in rectangles:
            x, y, width, height = rect
            rl.draw_rectangle(int(x), int(y), int(width), int(height), color)
    
    @staticmethod
    def draw_lines_batch(lines: List[tuple], color: rl.Color) -> None:
        """Efficiently draw multiple lines in one call"""
        for line in lines:
            x1, y1, x2, y2 = line
            rl.draw_line(int(x1), int(y1), int(x2), int(y2), color)
    
    @staticmethod
    def draw_circle_batch(circles: List[tuple], color: rl.Color) -> None:
        """Efficiently draw multiple circles in one call"""
        for circle in circles:
            x, y, radius = circle
            rl.draw_circle(int(x), int(y), int(radius), color)


def main():
    """Main entry point for the raylib UI implementation"""
    print("Initializing Raylib UI for Sunnypilot...")
    
    # Initialize UI configuration
    config = UIConfig()
    
    # Initialize raylib window
    rl.init_window(config.screen_width, config.screen_height, "Sunnypilot UI")
    rl.set_target_fps(config.target_fps)
    
    # Initialize UI system
    ui_system = RaylibUI(config)
    
    print("UI system initialized successfully!")
    print(f"Target FPS: {config.target_fps}")
    print(f"Screen Resolution: {config.screen_width}x{config.screen_height}")
    print("Press ESC to exit, D for debug mode, C for compact mode")
    
    try:
        while not rl.window_should_close():
            # Update UI system
            ui_system.update()
            
            # Begin drawing
            rl.begin_drawing()
            rl.clear_background(rl.BLACK)
            
            # Render UI
            screen_rect = rl.Rectangle(0, 0, config.screen_width, config.screen_height)
            ui_system.render(screen_rect)
            
            # Handle input
            if rl.is_key_pressed(rl.KeyboardKey.KEY_ESCAPE):
                break
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_D):
                ui_system.enable_debug_mode(not ui_system.state_manager.debug_mode)
            elif rl.is_key_pressed(rl.KeyboardKey.KEY_C):
                ui_system.toggle_compact_mode()
            
            # Display performance metrics
            metrics = ui_system.get_performance_metrics()
            perf_text = f"FPS: {metrics['fps']:.1f} | Render: {metrics['render_time_ms']:.1f}ms"
            rl.draw_text(perf_text, 10, 10, 16, rl.WHITE)
            
            if metrics['resource_warning']:
                warning_text = "RESOURCE WARNING!"
                rl.draw_text(warning_text, 10, 30, 16, rl.RED)
            
            # End drawing
            rl.end_drawing()
    
    except KeyboardInterrupt:
        print("UI shutdown requested")
    finally:
        # Cleanup
        rl.close_window()
        print("Raylib UI shutdown complete")


if __name__ == "__main__":
    main()