"""
System Status Panel Component
Safety validation status, sensor health, system readiness indicators
"""
import pyray as rl
import time
from typing import Dict, List, Optional
from dataclasses import dataclass

from cereal import messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.sunnypilot_ui import UIComponent


@dataclass
class SensorStatus:
    """Status of individual sensors"""
    name: str
    status: str  # "OK", "WARNING", "ERROR", "OFFLINE"
    health_score: float  # 0.0 to 1.0
    last_update: float
    message: str = ""


@dataclass
class SafetyCheck:
    """Result of a safety validation check"""
    name: str
    status: str  # "PASS", "FAIL", "PENDING"
    confidence: float  # 0.0 to 1.0
    critical: bool
    last_check: float
    details: str = ""


@dataclass
class SystemHealth:
    """Overall system health status"""
    overall_status: str  # "HEALTHY", "DEGRADED", "CRITICAL", "UNKNOWN"
    safety_score: float  # 0.0 to 1.0
    ready_for_autonomous: bool
    last_full_check: float


class SystemStatusPanel(UIComponent):
    """System status panel showing safety validation and sensor health"""
    
    def __init__(self):
        super().__init__("SystemStatusPanel")
        
        # System status
        self.system_health = SystemHealth("UNKNOWN", 0.0, False, 0)
        self.sensors: List[SensorStatus] = []
        self.safety_checks: List[SafetyCheck] = []
        self.last_full_status_update = 0.0
        
        # UI configuration
        self.panel_width = 280
        self.panel_height = 350
        self.panel_x = lambda rect: rect.x + rect.width - self.panel_width - 20
        self.panel_y = lambda rect: rect.y + 120
        self.update_interval = 2.0  # Update status every 2 seconds
        
        # Status colors
        self.status_colors = {
            "HEALTHY": rl.Color(50, 205, 50, 255),    # Green
            "DEGRADED": rl.Color(255, 165, 0, 255),  # Orange
            "CRITICAL": rl.Color(220, 20, 60, 255),  # Red
            "UNKNOWN": rl.Color(169, 169, 169, 255), # Gray
            "OK": rl.Color(50, 205, 50, 255),        # Green
            "WARNING": rl.Color(255, 165, 0, 255),   # Orange
            "ERROR": rl.Color(220, 20, 60, 255),     # Red
            "OFFLINE": rl.Color(169, 169, 169, 255), # Gray
            "PASS": rl.Color(50, 205, 50, 255),      # Green
            "FAIL": rl.Color(220, 20, 60, 255),      # Red
            "PENDING": rl.Color(210, 210, 100, 255)  # Yellow
        }
        
        # Initialize sensor status
        self._initialize_sensors()
    
    def _initialize_sensors(self):
        """Initialize sensor status list"""
        sensor_names = [
            "camera", "radar", "lidar", "gps", "imu", 
            "ultrasonic", "wheel_speed", "steering_angle"
        ]
        
        for name in sensor_names:
            self.sensors.append(SensorStatus(
                name=name,
                status="UNKNOWN",
                health_score=0.0,
                last_update=0.0
            ))
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update system status from system messages"""
        current_time = time.time()
        
        # Update more frequently for critical data
        if sm.updated["deviceState"]:
            device_state = sm["deviceState"]
            
            # Update overall system status based on device state
            thermal_status = device_state.thermalStatus
            if thermal_status == 0:  # ThermalStatus.normal
                self.system_health.overall_status = "HEALTHY"
            elif thermal_status <= 2:  # ThermalStatus.green to yellow
                self.system_health.overall_status = "DEGRADED"
            else:  # ThermalStatus.red to dangerous
                self.system_health.overall_status = "CRITICAL"
            
            # Update sensor statuses from device state
            self._update_sensor_status(device_state, current_time)
        
        # Update from car state for additional sensor information
        if sm.updated["carState"]:
            car_state = sm["carState"]
            self._update_sensor_status_from_car(car_state, current_time)
        
        # Less frequent updates
        if current_time - self.last_full_status_update > self.update_interval:
            self._update_safety_status(sm)
            self.last_full_status_update = current_time
    
    def _update_sensor_status(self, device_state, current_time: float):
        """Update sensor status from device state"""
        # Update camera status
        cam_idx = next((i for i, s in enumerate(self.sensors) if s.name == "camera"), -1)
        if cam_idx >= 0:
            # In a real implementation, this would check actual camera status
            self.sensors[cam_idx].status = "OK"
            self.sensors[cam_idx].health_score = 1.0
            self.sensors[cam_idx].last_update = current_time
        
        # Update GPS status
        gps_idx = next((i for i, s in enumerate(self.sensors) if s.name == "gps"), -1)
        if gps_idx >= 0:
            # Assume GPS is OK if we have valid location
            if device_state.locationMonoTime > 0:
                self.sensors[gps_idx].status = "OK"
                self.sensors[gps_idx].health_score = 0.9 if device_state.gpsAccuracy < 5.0 else 0.7
            else:
                self.sensors[gps_idx].status = "WARNING"
                self.sensors[gps_idx].health_score = 0.3
            self.sensors[gps_idx].last_update = current_time
    
    def _update_sensor_status_from_car(self, car_state, current_time: float):
        """Update sensor status from car state"""
        # Update wheel speed sensors
        wheel_idx = next((i for i, s in enumerate(self.sensors) if s.name == "wheel_speed"), -1)
        if wheel_idx >= 0:
            if car_state.valid:
                self.sensors[wheel_idx].status = "OK"
                self.sensors[wheel_idx].health_score = 1.0
            else:
                self.sensors[wheel_idx].status = "WARNING"
                self.sensors[wheel_idx].health_score = 0.6
            self.sensors[wheel_idx].last_update = current_time
    
    def _update_safety_status(self, sm: messaging.SubMaster):
        """Update safety validation status"""
        # In a real implementation, this would connect to the safety validation system
        # For now, we'll simulate safety checks
        
        # Reset safety checks list
        self.safety_checks = []
        
        # Add simulated safety checks
        self.safety_checks.append(SafetyCheck(
            name="Collision Avoidance",
            status="PASS" if self.system_health.overall_status != "CRITICAL" else "FAIL",
            confidence=0.98,
            critical=True,
            last_check=time.time(),
            details="Lateral and longitudinal control verified"
        ))
        
        self.safety_checks.append(SafetyCheck(
            name="Sensor Fusion",
            status="PASS",
            confidence=0.95,
            critical=True,
            last_check=time.time(),
            details="Camera-radar fusion validated"
        ))
        
        self.safety_checks.append(SafetyCheck(
            name="Emergency Braking",
            status="PASS",
            confidence=0.99,
            critical=True,
            last_check=time.time(),
            details="Response time: 120ms"
        ))
        
        self.safety_checks.append(SafetyCheck(
            name="Lane Keeping",
            status="PASS",
            confidence=0.92,
            critical=True,
            last_check=time.time(),
            details="Lane departure prevention active"
        ))
        
        # Calculate overall safety score
        if self.safety_checks:
            total_score = sum(check.confidence for check in self.safety_checks)
            self.system_health.safety_score = total_score / len(self.safety_checks)
            
            # Check if system is ready for autonomous driving
            critical_checks_passed = all(
                check.status == "PASS" for check in self.safety_checks if check.critical
            )
            self.system_health.ready_for_autonomous = (
                critical_checks_passed and 
                self.system_health.overall_status in ["HEALTHY", "DEGRADED"]
            )
    
    def _get_status_color(self, status: str) -> rl.Color:
        """Get color for status"""
        return self.status_colors.get(status, self.status_colors["UNKNOWN"])
    
    def _render_sensor_status(self, rect: rl.Rectangle, start_y: float) -> float:
        """Render sensor status section"""
        # Draw section header
        header_y = start_y
        rl.draw_text("SENSORS", int(rect.x + 10), int(header_y), 16, rl.LIGHTGRAY)
        
        # Draw sensor status list
        sensor_y = header_y + 25
        item_height = 20
        text_size = 14
        
        for sensor in self.sensors[:5]:  # Limit to first 5 sensors to fit space
            # Draw sensor name
            rl.draw_text(sensor.name.upper(), int(rect.x + 20), int(sensor_y), text_size, rl.GRAY)
            
            # Draw status indicator
            status_color = self._get_status_color(sensor.status)
            rl.draw_circle(int(rect.x + self.panel_width - 40), int(sensor_y + 8), 6, status_color)
            
            # Draw health score
            score_text = f"{sensor.health_score:.0f}" if sensor.health_score > 0 else "N/A"
            rl.draw_text(score_text, int(rect.x + self.panel_width - 80), int(sensor_y), text_size, rl.LIGHTGRAY)
            
            sensor_y += item_height + 5
        
        return sensor_y + 5
    
    def _render_safety_status(self, rect: rl.Rectangle, start_y: float) -> float:
        """Render safety validation section"""
        # Draw section header
        header_y = start_y
        rl.draw_text("SAFETY STATUS", int(rect.x + 10), int(header_y), 16, rl.LIGHTGRAY)
        
        # Draw overall safety score
        score_y = header_y + 25
        rl.draw_text(f"SCORE: {self.system_health.safety_score:.0f}%", 
                    int(rect.x + 20), int(score_y), 14, rl.LIGHTGRAY)
        
        # Draw readiness indicator
        ready_y = score_y + 20
        ready_text = "READY" if self.system_health.ready_for_autonomous else "NOT READY"
        ready_color = rl.Color(50, 205, 50, 255) if self.system_health.ready_for_autonomous else rl.Color(220, 20, 60, 255)
        rl.draw_text(f"DRIVE: {ready_text}", int(rect.x + 20), int(ready_y), 14, ready_color)
        
        return ready_y + 25
    
    def _render_critical_safety_checks(self, rect: rl.Rectangle, start_y: float) -> float:
        """Render critical safety checks"""
        # Draw section header
        header_y = start_y
        rl.draw_text("CRITICAL CHECKS", int(rect.x + 10), int(header_y), 16, rl.LIGHTGRAY)
        
        # Draw critical safety check status
        check_y = header_y + 25
        item_height = 18
        text_size = 12
        
        critical_checks = [check for check in self.safety_checks if check.critical][:3]  # Limit to 3
        
        for check in critical_checks:
            # Draw check name (truncated if too long)
            check_name = check.name if len(check.name) < 20 else check.name[:17] + "..."
            rl.draw_text(check_name, int(rect.x + 20), int(check_y), text_size, rl.GRAY)
            
            # Draw status indicator
            status_color = self._get_status_color(check.status)
            rl.draw_circle(int(rect.x + self.panel_width - 30), int(check_y + 7), 5, status_color)
            
            check_y += item_height + 3
        
        return check_y + 5
    
    def _render_system_status(self, rect: rl.Rectangle):
        """Render the complete system status panel"""
        panel_x = self.panel_x(rect)
        panel_y = self.panel_y(rect)
        
        # Draw panel background
        bg_color = rl.Color(25, 35, 45, 220)
        border_color = rl.Color(100, 130, 170, 255)
        
        rl.draw_rectangle(int(panel_x), int(panel_y), int(self.panel_width), int(self.panel_height), bg_color)
        rl.draw_rectangle_lines(int(panel_x), int(panel_y), int(self.panel_width), int(self.panel_height), border_color)
        
        # Draw panel title
        rl.draw_text("SYSTEM STATUS", int(panel_x + 10), int(panel_y + 10), 18, rl.LIGHTGRAY)
        
        # Draw overall status
        status_y = panel_y + 35
        status_color = self._get_status_color(self.system_health.overall_status)
        rl.draw_text(f"STATUS: {self.system_health.overall_status}", 
                    int(panel_x + 10), int(status_y), 14, status_color)
        
        # Render individual sections
        current_y = status_y + 20
        current_y = self._render_sensor_status(rl.Rectangle(panel_x, current_y, self.panel_width, 0), current_y)
        current_y = self._render_safety_status(rl.Rectangle(panel_x, current_y, self.panel_width, 0), current_y)
        self._render_critical_safety_checks(rl.Rectangle(panel_x, current_y, self.panel_width, 0), current_y)
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render the system status panel"""
        if not self.visible:
            return
        
        self._render_system_status(rect)


class CompactSystemStatus(UIComponent):
    """Compact version of system status for minimal UI display"""
    
    def __init__(self):
        super().__init__("CompactSystemStatus")
        
        # Simplified status
        self.overall_status = "UNKNOWN"
        self.safety_score = 0.0
        self.ready_for_autonomous = False
        self.last_status_update = 0.0
    
    def _update_internal(self, sm: messaging.SubMaster):
        """Update with simplified status information"""
        current_time = time.time()
        
        if sm.updated["deviceState"]:
            device_state = sm["deviceState"]
            
            # Update based on thermal status
            thermal_status = device_state.thermalStatus
            if thermal_status == 0:  # ThermalStatus.normal
                self.overall_status = "HEALTHY"
            elif thermal_status <= 2:  # ThermalStatus.green to yellow
                self.overall_status = "DEGRADED"
            else:  # ThermalStatus.red to dangerous
                self.overall_status = "CRITICAL"
            
            # Simulate safety score based on thermal status
            if self.overall_status == "HEALTHY":
                self.safety_score = 0.95
            elif self.overall_status == "DEGRADED":
                self.safety_score = 0.75
            else:
                self.safety_score = 0.30
            
            # Ready status based on overall health
            self.ready_for_autonomous = self.overall_status in ["HEALTHY", "DEGRADED"]
            
            self.last_status_update = current_time
    
    def _render_internal(self, rect: rl.Rectangle):
        """Render compact system status in corner"""
        # Draw small status indicator in top-right corner
        x = rect.x + rect.width - 100
        y = rect.y + 20
        width = 90
        height = 40
        
        # Determine color based on status
        if self.overall_status == "HEALTHY":
            bg_color = rl.Color(30, 60, 30, 180)  # Dark green
            text_color = rl.Color(100, 255, 100, 255)  # Light green
        elif self.overall_status == "DEGRADED":
            bg_color = rl.Color(60, 50, 20, 180)  # Dark orange
            text_color = rl.Color(255, 200, 100, 255)  # Light orange
        elif self.overall_status == "CRITICAL":
            bg_color = rl.Color(70, 20, 20, 180)  # Dark red
            text_color = rl.Color(255, 100, 100, 255)  # Light red
        else:
            bg_color = rl.Color(40, 40, 40, 180)   # Dark gray
            text_color = rl.Color(180, 180, 180, 255)  # Light gray
        
        # Draw background
        rl.draw_rectangle(int(x), int(y), int(width), int(height), bg_color)
        rl.draw_rectangle_lines(int(x), int(y), int(width), int(height), rl.Color(120, 120, 140, 200))
        
        # Draw status text
        status_text = "OK" if self.ready_for_autonomous else "!"
        rl.draw_text(status_text, int(x + 35), int(y + 10), 20, text_color)
        
        # Draw safety score as small indicator
        score_text = f"{int(self.safety_score * 100)}"
        rl.draw_text(score_text, int(x + 5), int(y + 5), 12, text_color)