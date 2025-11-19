"""
Enhanced System Status Component for Raylib UI
Displays validation metrics, system health, and performance data
"""
import pyray as rl
import time
from typing import Dict, Any, Optional, List
from enum import Enum
import numpy as np

from cereal import messaging
from openpilot.selfdrive.ui.components.system_status import SystemStatusPanel
from openpilot.selfdrive.common.validation_publisher import validation_metrics_publisher
from openpilot.selfdrive.common.thermal_management import thermal_manager
from openpilot.selfdrive.common.dynamic_adaptation import dynamic_adaptation, PerformanceMode
from openpilot.selfdrive.common.data_collector import data_collection_manager


class ValidationLevel(Enum):
    """Validation confidence levels for UI"""
    CRITICAL = "critical"
    WARNING = "warning" 
    CAUTION = "caution"
    NORMAL = "normal"
    HIGH = "high"


class EnhancedSystemStatusPanel(SystemStatusPanel):
    """Enhanced system status panel with validation and monitoring metrics"""

    def __init__(self, x: float = 10, y: float = 10, width: float = 300, height: float = 250):
        super().__init__()
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.name = "EnhancedSystemStatusPanel"
        
        # Initialize messaging for enhanced metrics
        try:
            self.sm = messaging.SubMaster(['validationMetrics', 'deviceState', 'carState'])
        except:
            self.sm = None
            
        # State tracking
        self.validation_metrics = {}
        self.thermal_metrics = {}
        self.performance_mode = PerformanceMode.MAX_PERFORMANCE
        self.data_stats = {}
        
        # Color scheme
        self.colors = {
            'background': rl.Color(30, 30, 40, 220),
            'border': rl.Color(100, 100, 120, 255),
            'text': rl.Color(220, 220, 220, 255),
            'critical': rl.Color(220, 20, 60, 255),      # Red
            'warning': rl.Color(255, 165, 0, 255),      # Orange
            'caution': rl.Color(255, 215, 0, 255),      # Gold
            'normal': rl.Color(100, 200, 100, 255),     # Green
            'high': rl.Color(50, 205, 50, 255),         # LimeGreen
            'title': rl.Color(176, 196, 222, 255),      # LightSteelBlue
            'progress_bg': rl.Color(50, 50, 60, 200),
            'progress_fill': rl.Color(70, 130, 180, 255)  # SteelBlue
        }

    def get_validation_level_color(self, confidence: float) -> rl.Color:
        """Get color based on validation confidence level"""
        if confidence < 0.3:
            return self.colors['critical']
        elif confidence < 0.6:
            return self.colors['warning']
        elif confidence < 0.8:
            return self.colors['caution']
        elif confidence < 0.95:
            return self.colors['normal']
        else:
            return self.colors['high']

    def get_validation_metrics_summary(self) -> Dict[str, Any]:
        """Get validation metrics from messaging"""
        if not self.sm or not self.sm.updated['validationMetrics']:
            return {
                'overall_confidence': 0.0,
                'system_safe': True,
                'lead_confidence_ok': True,
                'lane_confidence_ok': True,
                'weather_adjusted_confidence': 0.0,
                'temporal_consistency': 1.0
            }
        
        vm = self.sm['validationMetrics']
        
        return {
            'overall_confidence': vm.overallConfidence if hasattr(vm, 'overallConfidence') else 0.0,
            'system_safe': vm.isValid if hasattr(vm, 'isValid') else True,
            'lead_confidence_ok': getattr(vm.enhanced, 'leadConfidenceOk', True),
            'lane_confidence_ok': getattr(vm.enhanced, 'laneConfidenceOk', True),
            'weather_adjusted_confidence': getattr(vm.enhanced, 'speedAdjustedConfidence', 0.0),
            'temporal_consistency': getattr(vm.enhanced, 'temporalConsistency', 1.0)
        }

    def get_thermal_metrics_summary(self) -> Dict[str, Any]:
        """Get thermal metrics from thermal manager"""
        # Update thermal status if we have deviceState
        if self.sm and self.sm.updated['deviceState']:
            thermal_metrics = thermal_manager.update_thermal_status(self.sm['deviceState'])
            return thermal_metrics or {}
        else:
            # Fallback to getting metrics without deviceState
            return {
                'current_cpu_temp': 0.0,
                'current_memory_percent': 0.0,
                'current_cpu_percent': 0.0,
                'thermal_score': 1.0,
                'performance_scale': 1.0,
                'system_thermal_state': 'normal'
            }

    def get_performance_mode_summary(self) -> Dict[str, Any]:
        """Get performance mode and related metrics"""
        current_mode = dynamic_adaptation.get_current_mode()
        computation_factor = dynamic_adaptation.get_computation_factor()
        
        # Get system load data
        system_load = dynamic_adaptation.get_system_load()
        load_data = {}
        if system_load:
            load_data = {
                'cpu_percent': system_load.cpu_percent,
                'memory_percent': system_load.memory_percent,
                'cpu_temp': system_load.cpu_temp,
                'gpu_temp': system_load.gpu_temp
            }
        
        return {
            'current_mode': current_mode,
            'computation_factor': computation_factor,
            'system_load': load_data
        }

    def get_data_collection_stats(self) -> Dict[str, Any]:
        """Get data collection statistics"""
        try:
            stats = data_collection_manager.collector.get_collection_stats()
            return stats
        except:
            return {
                'performance_metrics': 0,
                'edge_cases': 0,
                'anomalies': 0
            }

    def update(self, sm: messaging.SubMaster):
        """Update the enhanced system status panel"""
        # Update messaging
        if self.sm:
            self.sm.update(0)
        
        # Update all metric summaries
        self.validation_metrics = self.get_validation_metrics_summary()
        self.thermal_metrics = self.get_thermal_metrics_summary()
        self.performance_mode_summary = self.get_performance_mode_summary()
        self.data_stats = self.get_data_collection_stats()

    def render(self, rect: rl.Rectangle):
        """Render the enhanced system status panel"""
        # Don't render if hidden
        if not self.visible or not self.enabled:
            return

        # Draw panel background and border
        self._draw_panel_background()

        # Draw section titles
        title_y = self.y + 10
        current_y = title_y + 25

        # Draw validation metrics
        current_y = self._draw_validation_section(current_y)

        # Draw thermal metrics
        current_y = self._draw_thermal_section(current_y)

        # Draw performance metrics
        current_y = self._draw_performance_section(current_y)

        # Draw data collection stats
        current_y = self._draw_data_collection_section(current_y)

        # Draw overall system health indicator
        self._draw_health_indicator()

    def _draw_panel_background(self):
        """Draw the panel background and border"""
        # Draw panel background
        rl.draw_rectangle(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['background'])
        # Draw panel border
        rl.draw_rectangle_lines(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['border'])

    def _draw_validation_section(self, y: float) -> float:
        """Draw validation metrics section"""
        # Draw section title
        rl.draw_text("VALIDATION", int(self.x + 10), int(y - 5), 14, self.colors['title'])
        
        y += 20
        
        # Draw overall confidence
        confidence = self.validation_metrics.get('overall_confidence', 0.0)
        confidence_color = self.get_validation_level_color(confidence)
        
        rl.draw_text(f"Conf: {confidence:.2f}", int(self.x + 10), int(y), 12, confidence_color)
        
        # Draw progress bar for confidence
        self._draw_progress_bar(self.x + 100, y, 180, 12, confidence, 1.0, confidence_color, self.colors['progress_bg'])
        
        y += 18
        
        # Draw system safety status
        system_safe = self.validation_metrics.get('system_safe', True)
        safety_color = self.colors['high'] if system_safe else self.colors['critical']
        safety_text = "SAFE" if system_safe else "UNSAFE"
        rl.draw_text(f"System: {safety_text}", int(self.x + 10), int(y), 12, safety_color)
        
        y += 18
        
        # Draw lead and lane confidence status
        lead_ok = self.validation_metrics.get('lead_confidence_ok', True)
        lane_ok = self.validation_metrics.get('lane_confidence_ok', True)
        
        lead_color = self.colors['high'] if lead_ok else self.colors['critical']
        lane_color = self.colors['high'] if lane_ok else self.colors['critical']
        
        rl.draw_text(f"Lead: {'OK' if lead_ok else 'BAD'}", int(self.x + 10), int(y), 12, lead_color)
        rl.draw_text(f"Lane: {'OK' if lane_ok else 'BAD'}", int(self.x + 80), int(y), 12, lane_color)
        
        y += 20
        return y

    def _draw_thermal_section(self, y: float) -> float:
        """Draw thermal metrics section"""
        rl.draw_text("THERMAL", int(self.x + 10), int(y - 5), 14, self.colors['title'])
        
        y += 20
        
        # Draw CPU temperature
        cpu_temp = self.thermal_metrics.get('current_cpu_temp', 0.0)
        cpu_color = self.get_validation_level_color(1.0 - min(1.0, cpu_temp / 90.0))  # Invert for temperature: lower is better
        rl.draw_text(f"CPU: {cpu_temp:.1f}C", int(self.x + 10), int(y), 12, cpu_color)
        
        y += 18
        
        # Draw memory usage
        mem_percent = self.thermal_metrics.get('current_memory_percent', 0.0)
        mem_color = self.get_validation_level_color(1.0 - min(1.0, mem_percent / 100.0))
        rl.draw_text(f"Mem: {mem_percent:.1f}%", int(self.x + 10), int(y), 12, mem_color)
        
        y += 18
        
        # Draw CPU usage
        cpu_percent = self.thermal_metrics.get('current_cpu_percent', 0.0)
        cpu_usage_color = self.get_validation_level_color(1.0 - min(1.0, cpu_percent / 100.0))
        rl.draw_text(f"CPU%: {cpu_percent:.1f}", int(self.x + 10), int(y), 12, cpu_usage_color)
        
        y += 18
        
        # Draw thermal score
        thermal_score = self.thermal_metrics.get('thermal_score', 1.0)
        thermal_color = self.get_validation_level_color(thermal_score)
        rl.draw_text(f"Score: {thermal_score:.2f}", int(self.x + 10), int(y), 12, thermal_color)
        
        y += 20
        return y

    def _draw_performance_section(self, y: float) -> float:
        """Draw performance metrics section"""
        rl.draw_text("PERFORMANCE", int(self.x + 10), int(y - 5), 14, self.colors['title'])
        
        y += 20
        
        # Draw performance mode
        mode = self.performance_mode_summary.get('current_mode', PerformanceMode.MAX_PERFORMANCE)
        mode_text = mode.name.replace('_', ' ')
        mode_color = self.colors['high'] if mode == PerformanceMode.MAX_PERFORMANCE else self.colors['warning']
        
        rl.draw_text(f"Mode: {mode_text}", int(self.x + 10), int(y), 12, mode_color)
        
        y += 18
        
        # Draw computation factor
        factor = self.performance_mode_summary.get('computation_factor', 1.0)
        factor_color = self.get_validation_level_color(factor)
        rl.draw_text(f"Comp: {factor:.2f}", int(self.x + 10), int(y), 12, factor_color)
        
        y += 18
        
        # Draw system load if available
        load_data = self.performance_mode_summary.get('system_load', {})
        if load_data:
            cpu_percent = load_data.get('cpu_percent', 0.0)
            mem_percent = load_data.get('memory_percent', 0.0)
            
            cpu_color = self.get_validation_level_color(1.0 - min(1.0, cpu_percent / 100.0))
            mem_color = self.get_validation_level_color(1.0 - min(1.0, mem_percent / 100.0))
            
            rl.draw_text(f"Load: {cpu_percent:.1f}%", int(self.x + 10), int(y), 12, cpu_color)
            rl.draw_text(f"Mem: {mem_percent:.1f}%", int(self.x + 100), int(y), 12, mem_color)
        
        y += 20
        return y

    def _draw_data_collection_section(self, y: float) -> float:
        """Draw data collection statistics section"""
        if y > self.y + self.height - 40:  # Don't draw if not enough space
            return y
            
        rl.draw_text("DATA", int(self.x + 10), int(y - 5), 14, self.colors['title'])
        
        y += 20
        
        # Draw collection stats
        perf_metrics = self.data_stats.get('performance_metrics', 0)
        edge_cases = self.data_stats.get('edge_cases', 0)
        anomalies = self.data_stats.get('anomalies', 0)
        
        rl.draw_text(f"P: {perf_metrics}", int(self.x + 10), int(y), 12, self.colors['text'])
        rl.draw_text(f"E: {edge_cases}", int(self.x + 70), int(y), 12, self.colors['text'])
        rl.draw_text(f"A: {anomalies}", int(self.x + 130), int(y), 12, self.colors['text'])
        
        y += 20
        return y

    def _draw_health_indicator(self):
        """Draw overall health indicator at the top right"""
        if not self.validation_metrics or not self.thermal_metrics:
            return
            
        # Calculate overall health score (simplified)
        conf_score = self.validation_metrics.get('overall_confidence', 0.0)
        thermal_score = self.thermal_metrics.get('thermal_score', 1.0)
        
        overall_score = (conf_score + thermal_score) / 2.0
        
        # Determine color based on overall score
        health_color = self.get_validation_level_color(overall_score)
        
        # Draw health indicator
        indicator_x = self.x + self.width - 30
        indicator_y = self.y + 10
        
        # Draw circle
        rl.draw_circle(int(indicator_x + 8), int(indicator_y + 8), 8, health_color)
        rl.draw_circle_lines(int(indicator_x + 8), int(indicator_y + 8), 8, self.colors['border'])

    def _draw_progress_bar(self, x: float, y: float, width: float, height: float, 
                          value: float, max_value: float, color: rl.Color, 
                          bg_color: rl.Color):
        """Draw a progress bar"""
        # Draw background
        rl.draw_rectangle(int(x), int(y), int(width), int(height), bg_color)
        
        # Draw filled portion
        if max_value > 0:
            progress_width = min(width, (value / max_value) * width)
            rl.draw_rectangle(int(x), int(y), int(progress_width), int(height), color)
        
        # Draw border
        rl.draw_rectangle_lines(int(x), int(y), int(width), int(height), self.colors['border'])


class CompactEnhancedStatus:
    """Compact version of enhanced status for resource-constrained scenarios"""

    def __init__(self, x: float = 10, y: float = 10):
        self.x = x
        self.y = y
        self.width = 200
        self.height = 80
        self.name = "CompactEnhancedStatus"
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster(['validationMetrics', 'deviceState'])
        except:
            self.sm = None
            
        # Colors
        self.colors = {
            'background': rl.Color(30, 30, 40, 200),
            'border': rl.Color(100, 100, 120, 200),
            'text': rl.Color(220, 220, 220, 255),
            'critical': rl.Color(220, 20, 60, 255),
            'warning': rl.Color(255, 165, 0, 255),
            'normal': rl.Color(100, 200, 100, 255),
            'safe': rl.Color(50, 205, 50, 255)
        }
        
        # Data storage
        self.validation_confidence = 0.0
        self.system_safe = True
        self.thermal_status = "normal"
        self.computation_factor = 1.0

    def update(self, sm: messaging.SubMaster):
        """Update compact status"""
        if self.sm:
            self.sm.update(0)
        
        # Update metrics
        if self.sm and self.sm.updated['validationMetrics']:
            vm = self.sm['validationMetrics']
            self.validation_confidence = vm.overallConfidence if hasattr(vm, 'overallConfidence') else 0.0
            self.system_safe = vm.isValid if hasattr(vm, 'isValid') else True
        
        # Update thermal status
        if self.sm and self.sm.updated['deviceState']:
            thermal_data = thermal_manager.update_thermal_status(self.sm['deviceState'])
            if thermal_data:
                self.thermal_status = thermal_data.get('system_thermal_state', 'normal')

        # Update computation factor
        self.computation_factor = dynamic_adaptation.get_computation_factor()

    def render(self, rect: rl.Rectangle):
        """Render compact status display"""
        if not self.visible or not self.enabled:
            return

        # Draw background
        rl.draw_rectangle(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['background'])
        rl.draw_rectangle_lines(int(self.x), int(self.y), int(self.width), int(self.height), self.colors['border'])

        y = self.y + 10
        
        # Draw validation confidence
        conf_color = self.colors['normal'] if self.validation_confidence >= 0.8 else \
                     self.colors['warning'] if self.validation_confidence >= 0.6 else self.colors['critical']
        
        rl.draw_text(f"Conf: {self.validation_confidence:.2f}", int(self.x + 5), int(y), 12, conf_color)
        
        y += 15
        
        # Draw system safety status
        safe_color = self.colors['safe'] if self.system_safe else self.colors['critical']
        safe_text = "SAFE" if self.system_safe else "UNSAFE"
        rl.draw_text(f"Sys: {safe_text}", int(self.x + 5), int(y), 12, safe_color)
        
        y += 15
        
        # Draw thermal status
        therm_color = self.colors['normal'] if self.thermal_status == 'normal' else \
                      self.colors['warning'] if self.thermal_status == 'caution' else self.colors['critical']
        rl.draw_text(f"Th: {self.thermal_status.upper()}", int(self.x + 5), int(y), 12, therm_color)
        
        y += 15
        
        # Draw computation factor
        comp_color = self.colors['normal'] if self.computation_factor >= 0.8 else self.colors['warning']
        rl.draw_text(f"Comp: {self.computation_factor:.2f}", int(self.x + 5), int(y), 12, comp_color)


# Export the enhanced components
__all__ = [
    "EnhancedSystemStatusPanel",
    "CompactEnhancedStatus",
    "ValidationLevel"
]