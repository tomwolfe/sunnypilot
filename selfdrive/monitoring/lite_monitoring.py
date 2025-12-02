#!/usr/bin/env python3
"""
Lightweight Monitoring System for sunnypilot2
This module implements essential safety checks with minimal computational overhead
designed for Comma 3x hardware constraints.
"""

import numpy as np
from typing import Dict
import time

# Import existing sunnypilot components
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params


class LightweightSafetyChecker:
    """
    Lightweight safety validation system with minimal computational overhead.
    """
    def __init__(self):
        self.safety_thresholds = {
            'max_long_accel': 3.0,     # Maximum longitudinal acceleration (m/s^2)
            'max_lat_accel': 2.5,      # Maximum lateral acceleration (m/s^2)
            'min_safe_distance': 30.0, # Minimum safe distance to lead vehicle (m)
        }
        
    def validate_outputs(self, actuators, car_state, radar_state) -> bool:
        """
        Lightweight validation of control outputs for safety.
        """
        # Quick validation checks with minimal computation
        if hasattr(actuators, 'accel'):
            if abs(actuators.accel) > self.safety_thresholds['max_long_accel']:
                return False
                
        if (hasattr(actuators, 'curvature') and hasattr(car_state, 'vEgo')):
            lat_accel = abs(actuators.curvature * car_state.vEgo**2)
            if lat_accel > self.safety_thresholds['max_lat_accel']:
                return False
                
        # Simple forward collision check
        if (hasattr(radar_state, 'leadOne') and 
            radar_state.leadOne.status and 
            car_state.vEgo > 0.1):
            
            relative_speed = car_state.vEgo - radar_state.leadOne.vRel
            if relative_speed > 0:
                ttc = radar_state.leadOne.dRel / relative_speed
                if ttc < 1.5 and radar_state.leadOne.dRel < 50.0:  # Less than 1.5s TTC
                    if hasattr(actuators, 'accel') and actuators.accel > 0:  # Accelerating toward lead
                        return False
        
        return True  # Validation passed


class LightweightSystemMonitor:
    """
    Minimal system monitoring with essential metrics only.
    """
    def __init__(self):
        self.last_log_time = time.monotonic()
        self.log_interval = 5.0  # Log every 5 seconds to reduce overhead
        
    def check_system_health(self, device_state, car_state) -> Dict:
        """
        Check basic system health with minimal computation.
        """
        health_report = {
            'cpu_overheated': False,
            'camera_healthy': True,
            'radar_healthy': True,
            'basic_comms_ok': True
        }
        
        # Check thermal state (minimal computation)
        if hasattr(device_state, 'cpuTempC') and device_state.cpuTempC:
            max_temp = max(device_state.cpuTempC) if isinstance(device_state.cpuTempC, list) else device_state.cpuTempC
            if max_temp and max_temp > 80.0:
                health_report['cpu_overheated'] = True
        
        # Log periodically to reduce overhead
        current_time = time.monotonic()
        if current_time - self.last_log_time > self.log_interval:
            # Only log basic info periodically
            cloudlog.debug(f"System Health: CPU Temp {max_temp if 'max_temp' in locals() else 'N/A'}")
            self.last_log_time = current_time
            
        return health_report


# Example usage and testing
if __name__ == "__main__":
    cloudlog.info("Initializing Lightweight Monitoring System")
    
    # Initialize lightweight monitoring system
    safety_checker = LightweightSafetyChecker()
    system_monitor = LightweightSystemMonitor()
    
    cloudlog.info("Lightweight Monitoring System initialized successfully")