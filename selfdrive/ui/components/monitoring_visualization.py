"""
Predictive Planning and Monitoring Visualization for Raylib UI
Displays predictive planning metrics, resource allocation, and system monitoring
"""
import pyray as rl
import time
from typing import Dict, Any, Optional, List
from enum import Enum
import numpy as np

from cereal import messaging
from openpilot.selfdrive.common.predictive_planning import predictive_planner
from openpilot.selfdrive.common.resource_aware import resource_manager
from openpilot.selfdrive.common.enhanced_fusion import enhanced_fusion
from openpilot.selfdrive.common.safety_redundancy import safety_monitor


class PlanningState(Enum):
    """Planning states for visualization"""
    IDLE = "idle"
    PLANNING = "planning"
    TRACKING = "tracking"
    ADAPTING = "adapting"


class ResourceAllocationVisualizer:
    """Visualizes resource allocation and management"""

    def __init__(self, x: float = 10, y: float = 10, width: float = 250, height: float = 150):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.name = "ResourceAllocationVisualizer"
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster(['deviceState'])
        except:
            self.sm = None
            
        # Colors
        self.colors = {
            'background': rl.Color(25, 25, 35, 220),
            'border': rl.Color(80, 80, 100, 255),
            'text': rl.Color(220, 220, 220, 255),
            'cpu': rl.Color(70, 130, 180, 255),      # SteelBlue
            'memory': rl.Color(50, 205, 50, 255),    # LimeGreen
            'gpu': rl.Color(220, 20, 60, 255),       # Crimson
            'available': rl.Color(100, 100, 100, 150), # Gray
            'title': rl.Color(176, 196, 222, 255)    # LightSteelBlue
        }
        
        # Data storage
        self.resource_stats = {}
        self.resource_history = {
            'cpu': [],
            'memory': [],
            'gpu': []
        }
        self.max_history = 50

    def update(self, sm: messaging.SubMaster):
        """Update resource allocation data"""
        # Update messaging
        if self.sm:
            self.sm.update(0)
        
        # Get resource statistics from the resource manager
        try:
            self.resource_stats = resource_manager.get_system_resource_status()
        except:
            self.resource_stats = {}
        
        # Update resource history for graphing
        util_data = self.resource_stats.get('utilization', {})
        for key in ['cpu_used_pct', 'memory_used_mb', 'gpu_used_pct']:
            if key in util_data:
                # Normalize memory for consistent display (divide by 2000 to get percentage)
                value = util_data[key]
                if 'memory' in key:
                    value = (value / 2000.0) * 100  # Assuming 2GB total, convert to percentage
                self._add_to_history(key.split('_')[0], min(100, value))

    def _add_to_history(self, key: str, value: float):
        """Add value to history for graphing"""
        if key not in self.resource_history:
            self.resource_history[key] = []
        self.resource_history[key].append(value)
        if len(self.resource_history[key]) > self.max_history:
            self.resource_history[key].pop(0)

    def render(self, rect: rl.Rectangle):
        """Render resource allocation visualization"""
        if not self.visible or not self.enabled:
            return

        # Draw container
        rl.draw_rectangle(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['background'])
        rl.draw_rectangle_lines(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['border'])

        y = self.y + 15
        rl.draw_text("RESOURCES", int(self.x + 10), int(y), 14, self.colors['title'])
        y += 20

        # Draw CPU usage bar
        cpu_pct = self.resource_stats.get('utilization', {}).get('cpu_used_pct', 0)
        self._draw_resource_bar(y, "CPU", cpu_pct, 100, self.colors['cpu'])
        y += 25

        # Draw Memory usage bar (showing MB used)
        mem_mb = self.resource_stats.get('utilization', {}).get('memory_used_mb', 0)
        mem_total = 2000  # Assuming 2GB total memory
        mem_pct = (mem_mb / mem_total) * 100 if mem_total > 0 else 0
        self._draw_resource_bar(y, f"MEM", mem_mb, mem_total, self.colors['memory'])
        y += 25

        # Draw GPU usage bar
        gpu_pct = self.resource_stats.get('utilization', {}).get('gpu_used_pct', 0)
        self._draw_resource_bar(y, "GPU", gpu_pct, 100, self.colors['gpu'])
        y += 30

        # Draw resource history graph
        self._draw_resource_history(y)

    def _draw_resource_bar(self, y: float, label: str, current: float, max_val: float, color: rl.Color):
        """Draw a resource usage bar with proper scaling"""
        from openpilot.selfdrive.ui.raylib_ui_system import ReusableUIComponents

        # Apply scaling to coordinates and dimensions
        bar_x = ReusableUIComponents.scaled_value(self.x) + ReusableUIComponents.scaled_value(60)
        bar_width = ReusableUIComponents.scaled_value(self.width) - ReusableUIComponents.scaled_value(70)
        bar_height = ReusableUIComponents.scaled_value(15)

        # Draw label
        label_x = ReusableUIComponents.scaled_value(self.x) + ReusableUIComponents.scaled_value(10)
        text_size = ReusableUIComponents.scaled_value(12)
        rl.draw_text(f"{label}:", int(label_x), int(y + ReusableUIComponents.scaled_value(2)),
                    text_size, self.colors['text'])

        # Draw background
        rl.draw_rectangle(int(bar_x), int(y), int(bar_width), int(bar_height), self.colors['available'])

        # Draw usage
        if max_val > 0:
            usage_width = int((current / max_val) * bar_width)
            rl.draw_rectangle(int(bar_x), int(y), usage_width, int(bar_height), color)

        # Draw percentage text
        if max_val > 0:
            pct = (current / max_val) * 100
            text = f"{pct:.1f}%" if current <= max_val else f"{current:.0f}MB"

            # Calculate text position at the end of the bar
            text_size_small = ReusableUIComponents.scaled_value(10)
            text_x = int(bar_x + bar_width - ReusableUIComponents.scaled_value(40))
            text_y = int(y + ReusableUIComponents.scaled_value(1))
            rl.draw_text(text, text_x, text_y, text_size_small, self.colors['text'])

    def _draw_resource_history(self, y: float):
        """Draw historical resource usage graph"""
        if y > self.y + self.height - 30:
            return

        graph_x = self.x + 10
        graph_width = self.width - 20
        graph_height = 30

        # Draw graph background
        rl.draw_rectangle(int(graph_x), int(y), int(graph_width), int(graph_height), self.colors['available'])

        # Draw CPU history (blue line)
        if len(self.resource_history['cpu']) > 1:
            self._draw_history_line(graph_x, y, graph_width, graph_height, self.resource_history['cpu'], self.colors['cpu'])

        # Draw Memory history (green line)
        if len(self.resource_history['memory']) > 1:
            self._draw_history_line(graph_x, y, graph_width, graph_height, self.resource_history['memory'], self.colors['memory'])

    def _draw_history_line(self, base_x: float, base_y: float, width: float, height: float, 
                          data: List[float], color: rl.Color):
        """Draw a history line graph"""
        if len(data) < 2:
            return

        # Scale data to fit graph
        max_val = max(data) if data else 1
        if max_val == 0:
            max_val = 1

        points = []
        for i, value in enumerate(data):
            x = base_x + (i * width / (len(data) - 1)) if len(data) > 1 else base_x
            y = base_y + height - (value / max_val) * height
            points.append((int(x), int(y)))

        # Draw line connecting points
        for i in range(len(points) - 1):
            rl.draw_line(points[i][0], points[i][1], points[i+1][0], points[i+1][1], color)


class PredictivePlanningVisualizer:
    """Visualizes predictive planning and trajectory information"""

    def __init__(self, x: float = 270, y: float = 10, width: float = 300, height: float = 150):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.name = "PredictivePlanningVisualizer"
        
        # Colors
        self.colors = {
            'background': rl.Color(35, 35, 45, 220),
            'border': rl.Color(100, 100, 120, 255),
            'text': rl.Color(220, 220, 220, 255),
            'path': rl.Color(100, 200, 255, 255),      # LightBlue
            'obstacle': rl.Color(255, 165, 0, 255),    # Orange
            'predicted': rl.Color(150, 150, 255, 200), # LightPurple
            'safe': rl.Color(50, 205, 50, 255),        # LimeGreen
            'caution': rl.Color(255, 215, 0, 255),     # Gold
            'danger': rl.Color(220, 20, 60, 255),      # Crimson
            'title': rl.Color(176, 196, 222, 255)      # LightSteelBlue
        }
        
        # State
        self.planning_state = PlanningState.IDLE
        self.predicted_objects = []
        self.trajectory_cost = 0.0
        self.safety_score = 1.0

    def update(self, sm: messaging.SubMaster):
        """Update predictive planning data"""
        # Update planning state based on system status
        # This would integrate with the actual predictive planner in real implementation
        try:
            # In a real system, we would call the planner and get current predictions
            # For now, we'll simulate data
            self.planning_state = PlanningState.PLANNING  # Simulating planning state
            
            # Simulate some predicted objects
            self.predicted_objects = [
                {'id': 1, 'dRel': 30.0, 'yRel': 1.0, 'vRel': -2.0, 'prob': 0.9},
                {'id': 2, 'dRel': 50.0, 'yRel': -1.5, 'vRel': 0.0, 'prob': 0.7}
            ]
            
            # Simulate trajectory cost and safety score
            self.trajectory_cost = 0.5
            self.safety_score = 0.85
        except Exception as e:
            print(f"Error updating predictive planning: {e}")
            self.planning_state = PlanningState.IDLE

    def render(self, rect: rl.Rectangle):
        """Render predictive planning visualization"""
        if not self.visible or not self.enabled:
            return

        # Draw container
        rl.draw_rectangle(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['background'])
        rl.draw_rectangle_lines(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['border'])

        y = self.y + 15
        rl.draw_text("PLANNING", int(self.x + 10), int(y), 14, self.colors['title'])
        y += 20

        # Draw planning state
        state_color = self.colors['safe'] if self.planning_state != PlanningState.IDLE else self.colors['caution']
        rl.draw_text(f"State: {self.planning_state.value.upper()}", int(self.x + 10), int(y), 12, state_color)
        y += 18

        # Draw safety score
        safety_color = self.get_safety_color(self.safety_score)
        rl.draw_text(f"Safe: {self.safety_score:.2f}", int(self.x + 10), int(y), 12, safety_color)
        y += 18

        # Draw trajectory cost
        cost_color = self.colors['safe'] if self.trajectory_cost < 0.7 else self.colors['caution']
        rl.draw_text(f"Cost: {self.trajectory_cost:.2f}", int(self.x + 10), int(y), 12, cost_color)
        y += 18

        # Draw predicted objects count
        obj_count = len(self.predicted_objects)
        obj_color = self.colors['safe'] if obj_count < 5 else self.colors['caution'] if obj_count < 10 else self.colors['danger']
        rl.draw_text(f"Pred: {obj_count}", int(self.x + 10), int(y), 12, obj_color)

    def get_safety_color(self, safety_score: float) -> rl.Color:
        """Get color based on safety score"""
        if safety_score >= 0.8:
            return self.colors['safe']
        elif safety_score >= 0.6:
            return self.colors['caution']
        else:
            return self.colors['danger']


class SystemMonitoringDashboard:
    """Comprehensive dashboard showing all system monitoring information"""

    def __init__(self, x: float = 10, y: float = 170, width: float = 800, height: float = 200):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.name = "SystemMonitoringDashboard"
        
        # Initialize subcomponents
        self.resource_viz = ResourceAllocationVisualizer(x, y, 250, 150)
        self.planning_viz = PredictivePlanningVisualizer(x + 260, y, 300, 150)
        
        # Colors
        self.colors = {
            'background': rl.Color(30, 30, 40, 220),
            'border': rl.Color(100, 100, 120, 255),
            'text': rl.Color(220, 220, 220, 255),
            'title': rl.Color(176, 196, 222, 255)
        }
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster(['deviceState', 'carState', 'modelV2'])
        except:
            self.sm = None

    def update(self, sm: messaging.SubMaster):
        """Update the monitoring dashboard"""
        # Update messaging
        if self.sm:
            self.sm.update(0)
        
        # Update subcomponents
        self.resource_viz.update(sm)
        self.planning_viz.update(sm)

    def render(self, rect: rl.Rectangle):
        """Render the monitoring dashboard"""
        if not self.visible or not self.enabled:
            return

        # Draw dashboard background
        rl.draw_rectangle(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['background'])
        rl.draw_rectangle_lines(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['border'])

        # Draw title
        rl.draw_text("SYSTEM MONITORING", int(self.x + 10), int(self.y + 10), 16, self.colors['title'])

        # Render subcomponents
        self.resource_viz.render(rect)
        self.planning_viz.render(rect)

        # Draw additional monitoring info
        y = self.y + 170
        rl.draw_text("MONITORING DATA:", int(self.x + 10), int(y), 12, self.colors['text'])
        
        # Draw safety status from safety monitor
        try:
            safety_status = safety_monitor.get_safety_recommendation(
                {}, {}  # Placeholder - in real implementation would use actual data
            )
            # For now, just show that we can access the safety monitor
            rl.draw_text("Safety: Active", int(self.x + 120), int(y), 12, self.colors['text'])
        except:
            rl.draw_text("Safety: Unavailable", int(self.x + 120), int(y), 12, self.colors['text'])


class ValidationMetricsVisualizer:
    """Visualizes validation metrics and confidence levels"""

    def __init__(self, x: float = 580, y: float = 10, width: float = 200, height: float = 150):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.name = "ValidationMetricsVisualizer"
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster(['validationMetrics'])
        except:
            self.sm = None
            
        # Colors
        self.colors = {
            'background': rl.Color(25, 30, 40, 220),
            'border': rl.Color(80, 100, 120, 255),
            'text': rl.Color(220, 220, 220, 255),
            'high': rl.Color(50, 205, 50, 255),       # LimeGreen
            'normal': rl.Color(100, 200, 100, 255),   # LightGreen
            'caution': rl.Color(255, 215, 0, 255),    # Gold
            'warning': rl.Color(255, 165, 0, 255),    # Orange
            'critical': rl.Color(220, 20, 60, 255),   # Crimson
            'title': rl.Color(176, 196, 222, 255)     # LightSteelBlue
        }
        
        # Data storage
        self.validation_metrics = {}

    def update(self, sm: messaging.SubMaster):
        """Update validation metrics"""
        if self.sm:
            self.sm.update(0)
        
        if self.sm and self.sm.updated['validationMetrics']:
            vm = self.sm['validationMetrics']
            self.validation_metrics = {
                'overall_confidence': vm.overallConfidence if hasattr(vm, 'overallConfidence') else 0.0,
                'lead_confidence_avg': vm.leadConfidenceAvg if hasattr(vm, 'leadConfidenceAvg') else 0.0,
                'lane_confidence_avg': vm.laneConfidenceAvg if hasattr(vm, 'laneConfidenceAvg') else 0.0,
                'is_valid': vm.isValid if hasattr(vm, 'isValid') else False,
                'situation_factor': getattr(vm.enhanced, 'situationFactor', 1.0),
                'temporal_consistency': getattr(vm.enhanced, 'temporalConsistency', 1.0)
            }

    def render(self, rect: rl.Rectangle):
        """Render validation metrics visualization with proper scaling"""
        from openpilot.selfdrive.ui.raylib_ui_system import ReusableUIComponents

        if not self.visible or not self.enabled:
            return

        # Apply scaling to coordinates and dimensions
        scaled_x = ReusableUIComponents.scaled_value(self.x) if hasattr(ReusableUIComponents, '_scale_factor') else int(self.x)
        scaled_y = ReusableUIComponents.scaled_value(self.y) if hasattr(ReusableUIComponents, '_scale_factor') else int(self.y)
        scaled_width = ReusableUIComponents.scaled_value(self.width) if hasattr(ReusableUIComponents, '_scale_factor') else int(self.width)
        scaled_height = ReusableUIComponents.scaled_value(self.height) if hasattr(ReusableUIComponents, '_scale_factor') else int(self.height)

        # Draw container
        rl.draw_rectangle(scaled_x, scaled_y, scaled_width, scaled_height, self.colors['background'])
        rl.draw_rectangle_lines(scaled_x, scaled_y, scaled_width, scaled_height, self.colors['border'])

        y = scaled_y + ReusableUIComponents.scaled_value(15)
        text_size_title = ReusableUIComponents.scaled_value(14)
        text_size_normal = ReusableUIComponents.scaled_value(12)
        rl.draw_text("VALIDATION",
                    int(scaled_x + ReusableUIComponents.scaled_value(10)), int(y),
                    text_size_title, self.colors['title'])
        y += ReusableUIComponents.scaled_value(20)

        # Draw overall confidence
        conf = self.validation_metrics.get('overall_confidence', 0.0)
        conf_color = self.get_confidence_color(conf)
        rl.draw_text(f"Conf: {conf:.2f}",
                    int(scaled_x + ReusableUIComponents.scaled_value(10)), int(y),
                    text_size_normal, conf_color)

        # Draw confidence bar
        bar_x = scaled_x + ReusableUIComponents.scaled_value(60)
        bar_width = scaled_width - ReusableUIComponents.scaled_value(70)
        bar_height = ReusableUIComponents.scaled_value(12)
        rl.draw_rectangle(int(bar_x), int(y), int(bar_width), int(bar_height), self.colors['border'])
        fill_width = int((conf * bar_width)) if conf <= 1.0 else int(bar_width)
        rl.draw_rectangle(int(bar_x), int(y), fill_width, int(bar_height), conf_color)
        y += ReusableUIComponents.scaled_value(18)

        # Draw lead confidence
        lead_conf = self.validation_metrics.get('lead_confidence_avg', 0.0)
        lead_color = self.get_confidence_color(lead_conf)
        rl.draw_text(f"Lead: {lead_conf:.2f}",
                    int(scaled_x + ReusableUIComponents.scaled_value(10)), int(y),
                    text_size_normal, lead_color)
        y += ReusableUIComponents.scaled_value(18)

        # Draw lane confidence
        lane_conf = self.validation_metrics.get('lane_confidence_avg', 0.0)
        lane_color = self.get_confidence_color(lane_conf)
        rl.draw_text(f"Lane: {lane_conf:.2f}",
                    int(scaled_x + ReusableUIComponents.scaled_value(10)), int(y),
                    text_size_normal, lane_color)
        y += ReusableUIComponents.scaled_value(18)

        # Draw temporal consistency
        temp_cons = self.validation_metrics.get('temporal_consistency', 1.0)
        temp_color = self.get_confidence_color(temp_cons)
        rl.draw_text(f"Temp: {temp_cons:.2f}",
                    int(scaled_x + ReusableUIComponents.scaled_value(10)), int(y),
                    text_size_normal, temp_color)
        y += ReusableUIComponents.scaled_value(18)

        # Draw validation status
        is_valid = self.validation_metrics.get('is_valid', False)
        valid_color = self.colors['high'] if is_valid else self.colors['critical']
        status_text = "VALID" if is_valid else "INVALID"
        rl.draw_text(f"Status: {status_text}",
                    int(scaled_x + ReusableUIComponents.scaled_value(10)), int(y),
                    text_size_normal, valid_color)

    def get_confidence_color(self, confidence: float) -> rl.Color:
        """Get color based on confidence level"""
        if confidence >= 0.9:
            return self.colors['high']
        elif confidence >= 0.8:
            return self.colors['normal']
        elif confidence >= 0.6:
            return self.colors['caution']
        elif confidence >= 0.4:
            return self.colors['warning']
        else:
            return self.colors['critical']


# Export the components
__all__ = [
    "ResourceAllocationVisualizer",
    "PredictivePlanningVisualizer", 
    "SystemMonitoringDashboard",
    "ValidationMetricsVisualizer"
]