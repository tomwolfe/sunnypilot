"""
Navigation Display Component
Route visualization, turn-by-turn guidance, and destination input
"""
import pyray as rl
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.sunnypilot_ui import UIComponent


@dataclass
class RoutePoint:
    """Represents a point along the navigation route"""
    lat: float
    lng: float
    distance: float  # Distance from current position in meters
    bearing: float   # Direction in degrees


@dataclass
class NavigationInstruction:
    """Represents a navigation maneuver"""
    maneuver_type: str  # "TURN_LEFT", "TURN_RIGHT", "STRAIGHT", etc.
    distance: float     # Distance to maneuver in meters
    road_name: str      # Name of road/destination
    modifier: str = ""  # Additional direction info


class NavigationDisplay(UIComponent):
    """Navigation system with route visualization and turn-by-turn guidance"""
    
    def __init__(self):
        super().__init__("NavigationDisplay")
        
        # Navigation state
        self.current_route: List[RoutePoint] = []
        self.current_instruction: Optional[NavigationInstruction] = None
        self.next_instruction: Optional[NavigationInstruction] = None
        self.route_progress: float = 0.0  # 0.0 to 1.0
        self.total_route_distance: float = 0.0
        self.distance_to_destination: float = 0.0
        
        # UI Elements
        self.nav_panel_width = 300
        self.nav_panel_height = 200
        self.nav_panel_x = lambda rect: rect.x + rect.width - self.nav_panel_width - 20
        self.nav_panel_y = 100
        
        # Colors
        self.route_color = rl.Color(100, 200, 255, 200)
        self.route_highlight_color = rl.Color(255, 100, 100, 200)
        self.maneuver_arrow_color = rl.Color(255, 255, 200, 255)
        
        # For route visualization
        self.route_points_for_display: List[Tuple[float, float]] = []
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update navigation data from system messages"""
        # Update with navigation data
        if sm.updated["navInstruction"]:
            nav_instr = sm["navInstruction"]
            
            # Update current instruction
            self.current_instruction = NavigationInstruction(
                maneuver_type=nav_instr.maneuver,
                distance=nav_instr.distance,
                road_name=nav_instr.roadName if nav_instr.roadName else "Destination"
            )
            
            self.distance_to_destination = nav_instr.distanceRemaining if nav_instr.distanceRemaining > 0 else 0.0
            self.route_progress = nav_instr.routeProgress if nav_instr.routeProgress > 0 else 0.0
        
        if sm.updated["navRoute"]:
            # Update route points (simplified for now)
            # In a real system, this would come from navRoute message
            pass
    
    def _simplify_route_for_display(self) -> List[Tuple[float, float]]:
        """Simplify route points for efficient rendering"""
        if not self.current_route:
            return []
        
        # For now, return a mock route for demonstration
        # In a real implementation, this would convert GPS coordinates 
        # to screen coordinates and simplify the route
        route_display = []
        for i in range(min(10, len(self.current_route))):
            # Mock conversion - in reality, this would project GPS coordinates to screen space
            x = 50 + (i * 20)
            y = 100 + (i * 15) % 100
            route_display.append((x, y))
        
        return route_display
    
    def _get_maneuver_symbol(self, maneuver_type: str) -> str:
        """Get visual symbol for maneuver type"""
        symbols = {
            "TURN_LEFT": "←",
            "TURN_RIGHT": "→",
            "STRAIGHT": "↑",
            "SLIGHT_LEFT": "⇠",
            "SLIGHT_RIGHT": "⇢",
            "SHARP_LEFT": "⇇",
            "SHARP_RIGHT": "⇉",
            "UTURN_LEFT": "↺",
            "UTURN_RIGHT": "↻",
            "ARRIVE": "⦿",
        }
        return symbols.get(maneuver_type, "?")
    
    def _render_route_visualization(self, rect: rl.Rectangle):
        """Render the route on the screen"""
        # Get simplified route for display
        route_points = self._simplify_route_for_display()
        
        if len(route_points) > 1:
            # Draw route lines
            for i in range(len(route_points) - 1):
                start_x, start_y = route_points[i]
                end_x, end_y = route_points[i + 1]
                
                # Draw line segment
                rl.draw_line(
                    int(start_x), int(start_y),
                    int(end_x), int(end_y),
                    self.route_color
                )
                
                # Draw route points
                rl.draw_circle(int(start_x), int(start_y), 3, self.route_color)
        
        # Highlight current position along route
        if route_points and self.route_progress > 0:
            # Calculate current position index based on progress
            current_idx = int(self.route_progress * len(route_points))
            current_idx = min(current_idx, len(route_points) - 1)
            
            if current_idx >= 0:
                pos_x, pos_y = route_points[current_idx]
                rl.draw_circle(int(pos_x), int(pos_y), 6, self.route_highlight_color)
    
    def _render_turn_by_turn(self, rect: rl.Rectangle):
        """Render turn-by-turn navigation instructions"""
        if not self.current_instruction:
            return

        # Position for navigation panel
        panel_x = self.nav_panel_x(rect)
        panel_y = self.nav_panel_y
        panel_width = self.nav_panel_width
        panel_height = self.nav_panel_height

        # Use reusable panel component
        from openpilot.selfdrive.ui.raylib_ui_system import ReusableUIComponents
        ReusableUIComponents.create_panel(int(panel_x), int(panel_y), int(panel_width), int(panel_height),
                                        "NAVIGATION",
                                        rl.Color(30, 40, 50, 220),  # Dark blue background
                                        rl.Color(120, 140, 180, 255))  # Blue border

        # Draw current maneuver
        if self.current_instruction:
            # Maneuver symbol
            symbol = self._get_maneuver_symbol(self.current_instruction.maneuver_type)
            rl.draw_text(symbol, int(panel_x + 20), int(panel_y + 45), 48, self.maneuver_arrow_color)

            # Maneuver text
            maneuver_text = f"{self.current_instruction.road_name}"
            rl.draw_text(maneuver_text, int(panel_x + 80), int(panel_y + 50), 20, rl.WHITE)

            # Distance
            distance_text = f"{self.current_instruction.distance:.0f}m"
            rl.draw_text(distance_text, int(panel_x + 80), int(panel_y + 75), 18, rl.LIGHTGRAY)
        
        # Draw progress bar
        progress_x = panel_x + 20
        progress_y = panel_y + 120
        progress_width = panel_width - 40
        progress_height = 10
        
        # Background
        rl.draw_rectangle(int(progress_x), int(progress_y), int(progress_width), int(progress_height), 
                         rl.Color(80, 80, 90, 200))
        
        # Filled portion
        filled_width = self.route_progress * progress_width
        rl.draw_rectangle(int(progress_x), int(progress_y), int(filled_width), int(progress_height), 
                         rl.Color(100, 200, 255, 200))
        
        # Border
        rl.draw_rectangle_lines(int(progress_x), int(progress_y), int(progress_width), int(progress_height), 
                               rl.Color(150, 150, 160, 200))
        
        # Progress text
        progress_text = f"{self.route_progress * 100:.0f}% - {self.distance_to_destination:.0f}m to dest"
        rl.draw_text(progress_text, int(panel_x + 10), int(panel_y + 140), 14, rl.LIGHTGRAY)
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render the navigation display"""
        if not self.visible:
            return
        
        # Render route visualization (would be integrated with camera view in real implementation)
        # For now, we'll just render the turn-by-turn panel
        self._render_turn_by_turn(rect)


class DestinationInputPanel(UIComponent):
    """Input panel for destination entry"""
    
    def __init__(self):
        super().__init__("DestinationInputPanel")
        
        # UI Elements
        self.input_active = False
        self.destination_text = ""
        self.destination_buffer = [0] * 128  # Buffer for text input
        self.input_max_length = 50
        
        # UI Dimensions
        self.input_width = 400
        self.input_height = 80
        self.input_x = lambda rect: (rect.width - self.input_width) / 2
        self.input_y = 50
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render destination input panel"""
        if not self.visible:
            return
        
        input_x = self.input_x(rect)
        input_y = self.input_y
        
        # Draw input panel background
        bg_color = rl.Color(40, 40, 50, 240)
        border_color = rl.Color(100, 150, 200, 255)
        
        rl.draw_rectangle(int(input_x), int(input_y), int(self.input_width), int(self.input_height), bg_color)
        rl.draw_rectangle_lines(int(input_x), int(input_y), int(self.input_width), int(self.input_height), border_color)
        
        # Draw title
        rl.draw_text("Enter Destination", int(input_x + 20), int(input_y + 10), 18, rl.LIGHTGRAY)
        
        # Draw input field
        field_x = input_x + 15
        field_y = input_y + 40
        field_width = self.input_width - 30
        field_height = 25
        
        rl.draw_rectangle(int(field_x), int(field_y), int(field_width), int(field_height), rl.Color(30, 30, 40, 255))
        rl.draw_rectangle_lines(int(field_x), int(field_y), int(field_width), int(field_height), rl.Color(100, 100, 120, 200))
        
        # Draw input text
        text_color = rl.WHITE if self.input_active else rl.GRAY
        rl.draw_text(self.destination_text, int(field_x + 5), int(field_y + 5), 18, text_color)
        
        # Draw cursor if active
        if self.input_active:
            text_width = rl.measure_text(self.destination_text, 18)
            cursor_x = field_x + 5 + text_width
            rl.draw_line(int(cursor_x), int(field_y + 5), int(cursor_x), int(field_y + 20), rl.WHITE)
        
        # Draw button
        button_x = input_x + self.input_width - 100
        button_y = input_y + 40
        button_width = 80
        button_height = 25
        
        rl.draw_rectangle(int(button_x), int(button_y), int(button_width), int(button_height), rl.Color(70, 100, 150, 255))
        rl.draw_rectangle_lines(int(button_x), int(button_y), int(button_width), int(button_height), rl.Color(120, 150, 200, 255))
        rl.draw_text("SET", int(button_x + 25), int(button_y + 5), 16, rl.WHITE)