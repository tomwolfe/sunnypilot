"""
Controls Interface Component
Engage/disengage autonomous mode, settings access, destination input
"""
import pyray as rl
import time
from typing import Callable, Optional
from dataclasses import dataclass

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.sunnypilot_ui import UIComponent, DrivingState


@dataclass
class ControlButton:
    """Represents a control button"""
    x: float
    y: float
    width: float
    height: float
    text: str
    action: str  # "engage", "disengage", "settings", "destination", etc.
    enabled: bool = True
    color: rl.Color = rl.Color(70, 70, 90, 255)


class ControlsInterface(UIComponent):
    """Main controls interface for autonomous driving system"""
    
    def __init__(self):
        super().__init__("ControlsInterface")
        
        # Control states
        self.autonomous_engaged = False
        self.manual_override_active = False
        self.system_ready = False
        
        # UI Elements
        self.control_buttons: list[ControlButton] = []
        self.button_width = 120
        self.button_height = 50
        self.button_spacing = 10
        
        # Callbacks
        self.engage_callback: Optional[Callable] = None
        self.disengage_callback: Optional[Callable] = None
        self.settings_callback: Optional[Callable] = None
        self.destination_callback: Optional[Callable] = None
        
        # Colors
        self.engage_color = rl.Color(50, 180, 50, 255)    # Green
        self.disengage_color = rl.Color(220, 60, 60, 255) # Red
        self.settings_color = rl.Color(70, 130, 180, 255) # Blue
        self.disabled_color = rl.Color(80, 80, 90, 200)   # Gray
        
        # Initialize control buttons
        self._initialize_buttons()
    
    def _initialize_buttons(self):
        """Initialize control buttons with positions"""
        # Calculate starting position (centered horizontally, near bottom)
        total_width = (self.button_width * 3) + (self.button_spacing * 2)
        start_x = lambda rect: (rect.width - total_width) / 2
        start_y = lambda rect: rect.height - 80
        
        # Create buttons
        self.control_buttons = [
            ControlButton(
                x=0, y=0, width=self.button_width, height=self.button_height,
                text="ENGAGE", action="engage", color=self.engage_color
            ),
            ControlButton(
                x=0, y=0, width=self.button_width, height=self.button_height,
                text="DISENGAGE", action="disengage", color=self.disengage_color
            ),
            ControlButton(
                x=0, y=0, width=self.button_width, height=self.button_height,
                text="SETTINGS", action="settings", color=self.settings_color
            )
        ]
        
        # Store lambda functions for later calculation
        self._start_x_func = start_x
        self._start_y_func = start_y
    
    def set_callbacks(self, engage_callback: Optional[Callable] = None,
                     disengage_callback: Optional[Callable] = None,
                     settings_callback: Optional[Callable] = None,
                     destination_callback: Optional[Callable] = None):
        """Set callback functions for control actions"""
        self.engage_callback = engage_callback
        self.disengage_callback = disengage_callback
        self.settings_callback = settings_callback
        self.destination_callback = destination_callback
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update control states from system messages"""
        # Update from selfdrive state
        if sm.updated["selfdriveState"]:
            selfdrive_state = sm["selfdriveState"]
            self.autonomous_engaged = selfdrive_state.enabled
            self.system_ready = selfdrive_state.state != 0  # Not OFF
        
        # Update from car state for manual override detection
        if sm.updated["carState"]:
            car_state = sm["carState"]
            # Check for manual override (brake, gas, or steering input)
            self.manual_override_active = (
                car_state.brakePressed or
                car_state.gasPressed or
                car_state.steeringPressed
            )
    
    def _update_button_positions(self, rect: rl.Rectangle):
        """Update button positions based on current rectangle"""
        start_x = self._start_x_func(rect)
        start_y = self._start_y_func(rect)
        
        for i, button in enumerate(self.control_buttons):
            button.x = start_x + (i * (self.button_width + self.button_spacing))
            button.y = start_y
            
            # Update button enabled state based on current system state
            if button.action == "engage":
                button.enabled = not self.autonomous_engaged and self.system_ready
            elif button.action == "disengage":
                button.enabled = self.autonomous_engaged
            else:
                button.enabled = True  # Settings and other buttons always enabled
    
    def _handle_button_click(self, button: ControlButton):
        """Handle button click based on action"""
        if not button.enabled:
            return
        
        if button.action == "engage" and self.engage_callback:
            self.engage_callback()
        elif button.action == "disengage" and self.disengage_callback:
            self.disengage_callback()
        elif button.action == "settings" and self.settings_callback:
            self.settings_callback()
        elif button.action == "destination" and self.destination_callback:
            self.destination_callback()
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render the controls interface"""
        if not self.visible:
            return

        # Update button positions
        self._update_button_positions(rect)

        # Use the reusable button component for each button
        from openpilot.selfdrive.ui.raylib_ui_system import ReusableUIComponents

        for button in self.control_buttons:
            # Use the reusable button component which handles rendering and click detection
            clicked = ReusableUIComponents.create_button(
                button.x, button.y, button.width, button.height,
                button.text, button.enabled
            )

            if clicked:
                self._handle_button_click(button)
    
    def is_engaged(self) -> bool:
        """Return whether autonomous driving is engaged"""
        return self.autonomous_engaged
    
    def is_ready(self) -> bool:
        """Return whether system is ready for engagement"""
        return self.system_ready


class EmergencyControls(UIComponent):
    """Emergency controls for critical situations"""
    
    def __init__(self):
        super().__init__("EmergencyControls")
        
        # Emergency state
        self.emergency_active = False
        self.emergency_button: Optional[ControlButton] = None
        
        # Initialize emergency button
        self._initialize_emergency_button()
    
    def _initialize_emergency_button(self):
        """Initialize emergency button"""
        self.emergency_button = ControlButton(
            x=100, y=100, width=150, height=80,
            text="EMERGENCY STOP", action="emergency_stop",
            color=rl.Color(220, 20, 60, 255)  # Red
        )
    
    def set_emergency_state(self, active: bool):
        """Set emergency state"""
        self.emergency_active = active
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update emergency state from system messages"""
        # Check for emergency state in selfdrive state
        if sm.updated["selfdriveState"]:
            selfdrive_state = sm["selfdriveState"]
            # In a real system, this would check for emergency state
            self.emergency_active = selfdrive_state.state == 4  # Emergency state
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render emergency controls"""
        if not self.visible or not self.emergency_active:
            return
        
        # Position emergency button in center of screen
        if self.emergency_button:
            self.emergency_button.x = (rect.width - self.emergency_button.width) / 2
            self.emergency_button.y = (rect.height - self.emergency_button.height) / 2
            
            # Draw emergency overlay
            overlay_color = rl.Color(220, 20, 60, 100)  # Semi-transparent red
            rl.draw_rectangle(int(rect.x), int(rect.y), int(rect.width), int(rect.height), overlay_color)
            
            # Draw emergency button
            button = self.emergency_button
            rl.draw_rectangle(
                int(button.x), int(button.y),
                int(button.width), int(button.height),
                button.color
            )
            
            rl.draw_rectangle_lines(
                int(button.x), int(button.y),
                int(button.width), int(button.height),
                rl.Color(255, 200, 200, 255)
            )
            
            # Draw button text centered
            text_width = rl.measure_text(button.text, 18)
            text_x = button.x + (button.width - text_width) / 2
            text_y = button.y + (button.height - 25) / 2
            rl.draw_text(button.text, int(text_x), int(text_y), 18, rl.WHITE)
            
            # Handle mouse clicks
            if rl.is_mouse_button_pressed(rl.MouseButton.MOUSE_LEFT_BUTTON):
                mouse_x, mouse_y = rl.get_mouse_x(), rl.get_mouse_y()
                
                button_rect = rl.Rectangle(button.x, button.y, button.width, button.height)
                
                if (button_rect.x <= mouse_x <= button_rect.x + button_rect.width and
                    button_rect.y <= mouse_y <= button_rect.y + button_rect.height):
                    # In a real system, this would trigger an emergency stop
                    print("EMERGENCY STOP TRIGGERED")


class DrivingModeIndicator(UIComponent):
    """Visual indicator of current driving mode"""
    
    def __init__(self):
        super().__init__("DrivingModeIndicator")
        
        # Mode information
        self.driving_state = DrivingState.DISENGAGED
        self.autonomous_level = 0  # 0-5 level of autonomy
        
        # UI configuration
        self.indicator_width = 200
        self.indicator_height = 60
        self.indicator_x = lambda rect: (rect.width - self.indicator_width) / 2
        self.indicator_y = 20
        
        # Colors for different states
        self.state_colors = {
            DrivingState.MANUAL: rl.Color(220, 20, 60, 255),      # Red
            DrivingState.ENGAGED: rl.Color(50, 205, 50, 255),    # Green
            DrivingState.DISENGAGED: rl.Color(70, 130, 180, 255), # Blue
            DrivingState.EMERGENCY: rl.Color(220, 20, 60, 255)   # Red
        }
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update driving mode from system messages"""
        if sm.updated["selfdriveState"]:
            selfdrive_state = sm["selfdriveState"]
            
            # Update driving state
            if selfdrive_state.state == 4:  # Emergency
                self.driving_state = DrivingState.EMERGENCY
            elif selfdrive_state.enabled:
                self.driving_state = DrivingState.ENGAGED
            else:
                self.driving_state = DrivingState.DISENGAGED
            
            # Update autonomy level (simplified)
            if selfdrive_state.enabled:
                self.autonomous_level = 2  # Partial automation
            else:
                self.autonomous_level = 0  # No automation
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render driving mode indicator"""
        if not self.visible:
            return
        
        x = self.indicator_x(rect)
        y = self.indicator_y
        
        # Get color based on current state
        color = self.state_colors.get(self.driving_state, self.state_colors[DrivingState.DISENGAGED])
        
        # Draw background
        rl.draw_rectangle(int(x), int(y), int(self.indicator_width), int(self.indicator_height), 
                         rl.Color(color.r, color.g, color.b, 180))
        rl.draw_rectangle_lines(int(x), int(y), int(self.indicator_width), int(self.indicator_height), 
                               rl.Color(255, 255, 255, 220))
        
        # Draw state text
        state_text = self.driving_state.value.upper()
        text_width = rl.measure_text(state_text, 20)
        text_x = x + (self.indicator_width - text_width) / 2
        text_y = y + (self.indicator_height - 25) / 2
        rl.draw_text(state_text, int(text_x), int(text_y), 20, rl.WHITE)
        
        # Draw autonomy level indicator
        level_text = f"L{self.autonomous_level}"
        level_width = rl.measure_text(level_text, 16)
        rl.draw_text(level_text, int(x + self.indicator_width - level_width - 10), int(y + 5), 16, rl.WHITE)