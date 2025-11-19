"""
Perception Visualization Component
Visualize detected objects, lanes, traffic signs in a non-distracting way
"""
import pyray as rl
import time
import math
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.sunnypilot_ui import UIComponent


@dataclass
class DetectedObject:
    """Represents a detected object from perception system"""
    id: int
    type: str  # "car", "pedestrian", "bicycle", "traffic_sign", "animal", etc.
    x: float   # Relative x position (meters from center)
    y: float   # Relative y position (meters ahead)
    z: float   # Relative z position (meters vertical)
    width: float
    length: float
    height: float
    velocity_x: float  # Velocity in x direction
    velocity_y: float  # Velocity in y direction
    confidence: float  # Detection confidence (0.0 to 1.0)
    age: float  # Age of detection in seconds


@dataclass
class LaneBoundary:
    """Represents a lane boundary"""
    type: str  # "solid", "dashed", "double"
    points: List[Tuple[float, float]]  # List of (x, y) points
    confidence: float
    reliability: float


@dataclass
class TrafficSign:
    """Represents a detected traffic sign"""
    type: str  # "stop", "yield", "speed_limit", "traffic_light", etc.
    x: float   # Relative position
    y: float   # Relative position
    confidence: float
    value: str = ""  # For signs with values (e.g., speed limit)


class PerceptionVisualization(UIComponent):
    """Visualization of perception system detections"""
    
    def __init__(self):
        super().__init__("PerceptionVisualization")
        
        # Perception data
        self.objects: List[DetectedObject] = []
        self.lanes: List[LaneBoundary] = []
        self.traffic_signs: List[TrafficSign] = []
        
        # Configuration
        self.visualization_enabled = True
        self.max_objects_to_display = 20  # Limit to prevent visual clutter
        self.max_render_distance = 100.0  # Only render objects within this distance
        
        # Colors for different object types
        self.object_colors = {
            "car": rl.Color(255, 100, 100, 200),      # Red
            "pedestrian": rl.Color(255, 200, 100, 200), # Orange
            "bicycle": rl.Color(100, 200, 255, 200),    # Blue
            "truck": rl.Color(200, 100, 255, 200),      # Purple
            "animal": rl.Color(100, 255, 100, 200),     # Green
            "traffic_sign": rl.Color(255, 255, 100, 200), # Yellow
        }
        
        # Lane colors
        self.lane_colors = {
            "solid": rl.Color(255, 255, 255, 200),
            "dashed": rl.Color(200, 200, 200, 150),
            "double": rl.Color(255, 255, 255, 200),
        }
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update perception data from system messages"""
        # Update from modelV2 (contains lane and object information)
        if sm.updated["modelV2"]:
            model = sm["modelV2"]
            
            # Update lane boundaries
            self._update_lanes(model)
            
            # Update detected objects (leadsV3 contains lead vehicles)
            if len(model.leadsV3) > 0:
                self._update_objects_from_leads(model.leadsV3)
        
        # Update from liveCalibration for road model information
        if sm.updated["liveCalibration"]:
            # Calibration data could be used for improving visualization
            pass
    
    def _update_lanes(self, model):
        """Update lane boundary data from model"""
        self.lanes = []
        
        # Extract lane lines from model
        if len(model.laneLines) >= 4:
            for i, lane_line in enumerate(model.laneLines[:4]):  # First 4 lines: left, ego-left, ego-right, right
                lane_type = "dashed"  # Default
                if i == 0 or i == 3:  # Outer lanes are typically solid
                    lane_type = "solid"
                
                points = [(point.x, point.y) for point in lane_line.points if point.x != 0 or point.y != 0]
                if points:
                    self.lanes.append(LaneBoundary(
                        type=lane_type,
                        points=points,
                        confidence=lane_line.prob,
                        reliability=lane_line.std
                    ))
    
    def _update_objects_from_leads(self, leads):
        """Update objects from lead vehicle detections"""
        self.objects = []
        
        for i, lead in enumerate(leads):
            if not lead.prob or lead.prob < 0.5:  # Skip low confidence detections
                continue
                
            self.objects.append(DetectedObject(
                id=i,
                type="car",
                x=lead.x[0],  # x position relative to ego
                y=lead.y[0],  # y position (along road)
                z=0.0,  # Simplified
                width=2.0,  # Typical car width
                length=4.5,  # Typical car length
                height=1.5,  # Typical car height
                velocity_x=lead.v[0] if lead.v else 0.0,  # Velocity
                velocity_y=0.0,  # Simplified
                confidence=lead.prob,
                age=0.0  # Simplified
            ))
    
    def _world_to_screen(self, x: float, y: float, rect: rl.Rectangle) -> Tuple[float, float]:
        """
        Convert world coordinates to screen coordinates for visualization.
        This is a simplified transform; in a real system this would use the camera calibration.
        """
        # Simplified projection: assume a perspective view
        # Origin (0,0) is at the bottom center of the screen
        screen_center_x = rect.x + rect.width / 2
        screen_bottom_y = rect.y + rect.height
        
        # Scale factors (these would come from camera calibration in a real system)
        x_scale = 20  # pixels per meter for lateral position
        y_scale = 5   # pixels per meter for longitudinal position
        
        # Apply perspective effect
        # Objects closer appear larger and higher on the screen
        perspective_factor = max(0.1, 1.0 - (y / 100.0))  # Scale based on distance
        
        screen_x = screen_center_x + x * x_scale
        screen_y = screen_bottom_y - (y * y_scale) * perspective_factor
        
        # Limit to screen bounds
        screen_x = max(rect.x, min(screen_x, rect.x + rect.width))
        screen_y = max(rect.y, min(screen_y, rect.y + rect.height))
        
        return screen_x, screen_y
    
    def _render_objects(self, rect: rl.Rectangle):
        """Render detected objects"""
        for obj in self.objects[:self.max_objects_to_display]:  # Limit displayed objects
            if obj.y > self.max_render_distance:  # Skip objects too far away
                continue
            
            # Convert world coordinates to screen coordinates
            screen_x, screen_y = self._world_to_screen(obj.x, obj.y, rect)
            
            # Determine object color
            color = self.object_colors.get(obj.type, rl.Color(200, 200, 200, 200))
            
            # Adjust color based on confidence (more transparent for low confidence)
            alpha = int(min(255, 100 + obj.confidence * 155))
            display_color = rl.Color(color.r, color.g, color.b, alpha)
            
            # Calculate size based on distance (objects closer appear larger)
            size_factor = max(0.3, 1.0 - (obj.y / 50.0))  # Objects closer are larger
            width = max(5, obj.width * 3 * size_factor)   # Scale width
            height = max(5, obj.length * 3 * size_factor) # Scale length
            
            # Draw object as rectangle
            obj_rect = rl.Rectangle(
                screen_x - width / 2,
                screen_y - height / 2,
                width,
                height
            )
            
            rl.draw_rectangle_rec(obj_rect, display_color)
            rl.draw_rectangle_lines_ex(obj_rect, 1, rl.Color(display_color.r, display_color.g, display_color.b, 255))
            
            # Draw velocity vector
            if abs(obj.velocity_x) > 0.1 or abs(obj.velocity_y) > 0.1:
                vel_x = obj.velocity_x * 2  # Scale velocity for visibility
                vel_y = -obj.velocity_y * 2  # Negative because screen Y increases downward
                rl.draw_line(
                    int(screen_x), int(screen_y),
                    int(screen_x + vel_x), int(screen_y + vel_y),
                    rl.Color(255, 255, 255, 200)
                )
    
    def _render_lanes(self, rect: rl.Rectangle):
        """Render lane boundaries with optimized drawing"""
        from openpilot.selfdrive.ui.raylib_ui_system import EfficientDrawingRoutines

        for lane in self.lanes:
            if len(lane.points) < 2:
                continue

            # Convert world coordinates to screen coordinates for all points
            screen_points = []
            for point in lane.points:
                screen_x, screen_y = self._world_to_screen(point[0], point[1], rect)
                screen_points.append((screen_x, screen_y))

            # Use the optimized batch drawing routine for lane lines
            is_dashed = lane.type == "dashed"
            EfficientDrawingRoutines.draw_lane_lines_batch(
                screen_points,
                lane.confidence,
                is_dashed
            )
    
    def _render_traffic_signs(self, rect: rl.Rectangle):
        """Render traffic signs"""
        for sign in self.traffic_signs:
            screen_x, screen_y = self._world_to_screen(sign.x, sign.y, rect)
            
            # Draw sign marker (triangle for stop, circle for yield, etc.)
            sign_color = self.object_colors.get("traffic_sign", rl.Color(255, 255, 100, 200))
            
            if sign.type == "stop":
                # Draw octagon for stop sign
                self._draw_octagon(screen_x, screen_y, 15, sign_color)
            elif sign.type == "yield":
                # Draw triangle for yield sign
                self._draw_triangle(screen_x, screen_y, 15, sign_color)
            elif sign.type == "speed_limit":
                # Draw circle for speed limit sign
                rl.draw_circle(int(screen_x), int(screen_y), 12, sign_color)
                rl.draw_circle_lines(int(screen_x), int(screen_y), 12, rl.Color(0, 0, 0, 255))
            
            # Draw sign text if available
            if sign.value:
                rl.draw_text(sign.value, int(screen_x - 15), int(screen_y - 25), 14, rl.Color(0, 0, 0, 255))
    
    def _draw_triangle(self, center_x: float, center_y: float, size: float, color: rl.Color):
        """Draw an equilateral triangle pointing up"""
        height = size * math.sqrt(3) / 2
        
        # Calculate triangle vertices
        x1, y1 = center_x, center_y - height / 2  # Top
        x2, y2 = center_x - size / 2, center_y + height / 2  # Bottom left
        x3, y3 = center_x + size / 2, center_y + height / 2  # Bottom right
        
        # Draw filled triangle
        rl.draw_triangle(rl.Vector2(x1, y1), rl.Vector2(x2, y2), rl.Vector2(x3, y3), color)
        rl.draw_triangle_lines(rl.Vector2(x1, y1), rl.Vector2(x2, y2), rl.Vector2(x3, y3), rl.Color(0, 0, 0, 255))
    
    def _draw_octagon(self, center_x: float, center_y: float, size: float, color: rl.Color):
        """Draw an octagon (for stop signs)"""
        points = []
        for i in range(8):
            angle = (i * 45) * math.pi / 180  # Convert to radians
            x = center_x + size * math.cos(angle)
            y = center_y + size * math.sin(angle)
            points.append(rl.Vector2(x, y))
        
        # Draw filled octagon
        for i in range(8):
            rl.draw_triangle(
                rl.Vector2(center_x, center_y),
                points[i],
                points[(i + 1) % 8],
                color
            )
        
        # Draw outline
        for i in range(8):
            next_i = (i + 1) % 8
            rl.draw_line(int(points[i].x), int(points[i].y), 
                        int(points[next_i].x), int(points[next_i].y), 
                        rl.Color(0, 0, 0, 255))
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render perception visualization with improved resource management"""
        if not self.visible or not self.visualization_enabled:
            return

        # Render lane boundaries first (background)
        self._render_lanes(rect)

        # Render traffic signs
        self._render_traffic_signs(rect)

        # Render detected objects last (foreground)
        self._render_objects(rect)

    def _render_objects(self, rect: rl.Rectangle):
        """Render detected objects with proper visualization and LOD"""
        from openpilot.selfdrive.ui.raylib_ui_system import EfficientDrawingRoutines, ReusableUIComponents

        # Use the optimized drawing routine for multiple vehicles
        screen_transform_func = lambda x, y: self._world_to_screen(x, y, rect)

        # Convert our objects to the format expected by the drawing routine
        # Apply distance filtering and confidence threshold
        vehicle_data = []
        for obj in self.objects[:self.max_objects_to_display]:  # Limit displayed objects
            if obj.y > self.max_render_distance:  # Skip objects too far away
                continue
            if obj.confidence < 0.3:  # Skip low confidence detections
                continue

            vehicle_data.append({
                'x': obj.x,
                'y': obj.y,
                'width': obj.width,
                'length': obj.length,
                'type': obj.type,
                'prob': obj.confidence
            })

        # Use the optimized batch drawing routine
        EfficientDrawingRoutines.draw_vehicle_batch(vehicle_data, screen_transform_func)

        # Draw velocity vectors separately with optimization
        high_confidence_objects = [obj for obj in self.objects[:self.max_objects_to_display]
                                  if obj.y <= self.max_render_distance and obj.confidence >= 0.5]

        if len(high_confidence_objects) <= 10:  # Only draw vectors for limited objects to save resources
            for obj in high_confidence_objects:
                # Convert world coordinates to screen coordinates
                screen_x, screen_y = self._world_to_screen(obj.x, obj.y, rect)

                # Draw velocity vector
                if abs(obj.velocity_x) > 0.1 or abs(obj.velocity_y) > 0.1:
                    vel_x = obj.velocity_x * 2  # Scale velocity for visibility
                    vel_y = -obj.velocity_y * 2  # Negative because screen Y increases downward

                    # Use scaled values if available
                    scaled_screen_x = ReusableUIComponents.scaled_value(screen_x) if hasattr(ReusableUIComponents, '_scale_factor') else int(screen_x)
                    scaled_screen_y = ReusableUIComponents.scaled_value(screen_y) if hasattr(ReusableUIComponents, '_scale_factor') else int(screen_y)
                    scaled_vel_x = ReusableUIComponents.scaled_value(vel_x) if hasattr(ReusableUIComponents, '_scale_factor') else int(vel_x)
                    scaled_vel_y = ReusableUIComponents.scaled_value(vel_y) if hasattr(ReusableUIComponents, '_scale_factor') else int(vel_y)

                    rl.draw_line(
                        scaled_screen_x, scaled_screen_y,
                        scaled_screen_x + scaled_vel_x, scaled_screen_y + scaled_vel_y,
                        rl.Color(255, 255, 255, 200)
                    )


class PerceptionDebugOverlay(UIComponent):
    """Debug overlay showing perception system status"""
    
    def __init__(self):
        super().__init__("PerceptionDebugOverlay")
        
        # Debug information
        self.detection_count = 0
        self.average_confidence = 0.0
        self.update_frequency = 0.0
        self.last_update_time = 0.0
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update debug statistics"""
        current_time = time.time()
        
        # Update perception data
        if sm.updated["modelV2"]:
            model = sm["modelV2"]
            
            # Count high-confidence detections
            count = 0
            total_confidence = 0.0
            
            # Count lead vehicle detections
            for lead in model.leadsV3:
                if lead.prob and lead.prob > 0.5:
                    count += 1
                    total_confidence += lead.prob if lead.prob else 0.0
            
            self.detection_count = count
            self.average_confidence = total_confidence / count if count > 0 else 0.0
            
            # Update frequency
            if self.last_update_time > 0:
                time_diff = current_time - self.last_update_time
                if time_diff > 0:
                    self.update_frequency = 1.0 / time_diff
            
            self.last_update_time = current_time
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render debug overlay"""
        if not self.visible:
            return
        
        # Draw debug panel in top-left corner
        x = rect.x + 10
        y = rect.y + 10
        width = 200
        height = 80
        
        # Semi-transparent background
        bg_color = rl.Color(20, 30, 40, 180)
        rl.draw_rectangle(int(x), int(y), int(width), int(height), bg_color)
        rl.draw_rectangle_lines(int(x), int(y), int(width), int(height), rl.Color(100, 120, 150, 200))
        
        # Draw debug text
        text_color = rl.Color(200, 220, 255, 255)
        rl.draw_text(f"Detections: {self.detection_count}", int(x + 5), int(y + 5), 16, text_color)
        rl.draw_text(f"Confidence: {self.average_confidence:.2f}", int(x + 5), int(y + 25), 16, text_color)
        rl.draw_text(f"Freq: {self.update_frequency:.1f}Hz", int(x + 5), int(y + 45), 16, text_color)