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
            'max_steering_rate': 0.5,  # Maximum steering rate (rad/s) - adjust based on vehicle
            'max_steering_angle': 1.0, # Maximum steering angle (rad) - adjust based on vehicle
            'min_safe_ttc': 2.0,       # Minimum safe time-to-collision (s) for active intervention
            'min_safe_distance': 30.0, # Minimum safe distance to lead vehicle (m)
        }
        # Initialize tracking variables for steering rate calculation
        self._prev_steer = 0.0
        self._prev_time = time.monotonic()

    def validate_outputs(self, actuators, car_state, radar_state) -> Dict:
        """
        Lightweight validation of control outputs for safety.
        Returns a detailed report instead of just True/False.
        """
        safety_report = {
            'safe': True,
            'violations': [],
            'recommended_action': 'continue'  # Options: 'continue', 'decelerate', 'disengage'
        }

        # Quick validation checks with minimal computation
        if hasattr(actuators, 'accel'):
            if abs(actuators.accel) > self.safety_thresholds['max_long_accel']:
                safety_report['safe'] = False
                safety_report['violations'].append('long_accel_limit_exceeded')
                safety_report['recommended_action'] = 'decelerate'

        if (hasattr(actuators, 'curvature') and hasattr(car_state, 'vEgo')):
            lat_accel = abs(actuators.curvature * car_state.vEgo**2)
            if lat_accel > self.safety_thresholds['max_lat_accel']:
                safety_report['safe'] = False
                safety_report['violations'].append('lat_accel_limit_exceeded')
                safety_report['recommended_action'] = 'decelerate'

        # Check steering angle magnitude
        if hasattr(actuators, 'steer') and abs(actuators.steer) > self.safety_thresholds['max_steering_angle']:
            safety_report['safe'] = False
            safety_report['violations'].append('steering_angle_limit_exceeded')
            safety_report['recommended_action'] = 'disengage'

        # Check steering rate limit (change in steering command)
        if hasattr(self, '_prev_steer') and hasattr(self, '_prev_time') and hasattr(actuators, 'steer'):
            current_time = time.monotonic()
            time_delta = current_time - self._prev_time if hasattr(self, '_prev_time') else 0.01
            steering_rate = abs(actuators.steer - self._prev_steer) / max(time_delta, 0.01)  # Use a small dt to avoid division by zero
            if steering_rate > self.safety_thresholds['max_steering_rate']:
                safety_report['safe'] = False
                safety_report['violations'].append('steering_rate_limit_exceeded')
                safety_report['recommended_action'] = 'disengage'

        # Store current steering and time for next iteration
        if hasattr(actuators, 'steer'):
            self._prev_steer = actuators.steer
            self._prev_time = time.monotonic()

        # Enhanced forward collision check - handles all lead vehicle scenarios
        if (hasattr(radar_state, 'leadOne') and
            radar_state.leadOne.status and
            car_state.vEgo > 0.1):

            # Calculate relative speed: positive means approaching, negative means moving away
            relative_speed = car_state.vEgo - radar_state.leadOne.vRel

            # Check for potential collision regardless of lead vehicle's speed
            # If we're approaching (relative_speed > 0) OR lead vehicle is very slow/stopped
            if relative_speed > 0.1 or radar_state.leadOne.vRel < 0.1:
                # Calculate time-to-collision differently based on scenario
                if relative_speed > 0.1:  # Moving toward lead vehicle
                    ttc = radar_state.leadOne.dRel / relative_speed if relative_speed > 0.1 else float('inf')
                else:  # Lead vehicle is stationary or moving very slowly
                    ttc = radar_state.leadOne.dRel / car_state.vEgo if car_state.vEgo > 0.1 else float('inf')

                # If either TTC is below threshold OR distance is below safe distance (whichever condition triggers first)
                if ttc < self.safety_thresholds['min_safe_ttc'] or radar_state.leadOne.dRel < self.safety_thresholds['min_safe_distance']:
                    # This is a safety-critical situation regardless of current acceleration
                    safety_report['safe'] = False
                    safety_report['violations'].append('forward_collision_imminent')
                    safety_report['recommended_action'] = 'decelerate'

        return safety_report

    def trigger_fail_safe(self, safety_report: Dict) -> Dict:
        """
        Trigger appropriate fail-safe action based on safety report.
        """
        if not safety_report['safe']:
            if safety_report['recommended_action'] == 'disengage':
                # Return commands to disengage control
                return {
                    'acceleration': 0.0,  # Neutral acceleration
                    'steering': 0.0,      # Center steering
                    'enabled': False      # Disable control
                }
            elif safety_report['recommended_action'] == 'decelerate':
                # Return commands to apply emergency deceleration
                return {
                    'acceleration': -3.0,  # Apply braking
                    'steering': 0.0,       # Maintain current steering
                    'enabled': True        # Keep enabled but with safe commands
                }
        # If safe, no intervention needed
        return {
            'acceleration': None,  # Don't override
            'steering': None,      # Don't override
            'enabled': True        # Continue normal operation
        }


class LightweightSystemMonitor:
    """
    Minimal system monitoring with essential metrics only.
    """
    def __init__(self):
        self.last_log_time = time.monotonic()
        self.log_interval = 5.0  # Log every 5 seconds to reduce overhead
        self.health_thresholds = {
            'max_cpu_temp': 80.0,   # Maximum CPU temperature (°C)
            'max_gpu_temp': 85.0,   # Maximum GPU temperature (°C)
            'min_ram_free': 0.1,    # Minimum free RAM fraction
            'max_cpu_usage': 0.95,  # Maximum CPU usage fraction
        }

    def check_system_health(self, device_state, car_state) -> Dict:
        """
        Check comprehensive system health with minimal computation.
        """
        health_report = {
            'cpu_overheated': False,
            'gpu_overheated': False,
            'ram_low': False,
            'cpu_usage_high': False,
            'camera_healthy': True,
            'radar_healthy': True,
            'can_bus_ok': True,
            'network_connected': True,
            'disk_space_ok': True
        }

        # Check thermal state (CPU and GPU)
        if hasattr(device_state, 'cpuTempC') and device_state.cpuTempC:
            max_cpu_temp = max(device_state.cpuTempC) if isinstance(device_state.cpuTempC, list) else device_state.cpuTempC
            if max_cpu_temp and max_cpu_temp > self.health_thresholds['max_cpu_temp']:
                health_report['cpu_overheated'] = True

        if hasattr(device_state, 'gpuTempC') and device_state.gpuTempC:
            max_gpu_temp = max(device_state.gpuTempC) if isinstance(device_state.gpuTempC, list) else device_state.gpuTempC
            if max_gpu_temp and max_gpu_temp > self.health_thresholds['max_gpu_temp']:
                health_report['gpu_overheated'] = True

        # Check CPU usage if available
        if hasattr(device_state, 'cpuUsagePercent') and device_state.cpuUsagePercent:
            cpu_usage = device_state.cpuUsagePercent
            if cpu_usage > self.health_thresholds['max_cpu_usage']:
                health_report['cpu_usage_high'] = True

        # Check RAM usage if available
        if hasattr(device_state, 'memoryUsagePercent') and device_state.memoryUsagePercent:
            ram_usage = device_state.memoryUsagePercent
            if (100 - ram_usage) / 100.0 < self.health_thresholds['min_ram_free']:  # Invert since memoryUsagePercent is used%
                health_report['ram_low'] = True

        # Additional checks for CAN bus, disk, etc. if available
        if hasattr(device_state, 'canMonoTimes') and device_state.canMonoTimes:
            # Simple check: if we have old CAN times, consider it potentially unhealthy
            # Check if we have recent CAN messages by looking at the timestamps in canMonoTimes
            if hasattr(device_state, 'canMonoTimes') and device_state.canMonoTimes:
                # Get the most recent CAN time from the list
                recent_can_time = device_state.canMonoTimes[-1] if len(device_state.canMonoTimes) > 0 else 0
                if time.monotonic() - recent_can_time > 2.0:  # 2 seconds without CAN
                    health_report['can_bus_ok'] = False

        # Check disk space if available
        if hasattr(device_state, 'freeSpacePercent') and device_state.freeSpacePercent:
            if device_state.freeSpacePercent < 0.05:  # Less than 5% free
                health_report['disk_space_ok'] = False

        # Log periodically to reduce overhead
        current_time = time.monotonic()
        if current_time - self.last_log_time > self.log_interval:
            # Only log basic info periodically
            cpu_temp = max(device_state.cpuTempC) if hasattr(device_state, 'cpuTempC') and device_state.cpuTempC else 'N/A'
            gpu_temp = max(device_state.gpuTempC) if hasattr(device_state, 'gpuTempC') and device_state.gpuTempC else 'N/A'
            cpu_usage_val = device_state.cpuUsagePercent if hasattr(device_state, 'cpuUsagePercent') and device_state.cpuUsagePercent else 'N/A'
            ram_usage_val = device_state.memoryUsagePercent if hasattr(device_state, 'memoryUsagePercent') and device_state.memoryUsagePercent else 'N/A'

            cloudlog.debug(f"System Health: CPU {cpu_temp}, "
                          f"GPU {gpu_temp}, "
                          f"CPU% {cpu_usage_val}, "
                          f"RAM% {ram_usage_val}")
            self.last_log_time = current_time

        return health_report


# Example usage and testing
if __name__ == "__main__":
    cloudlog.info("Initializing Lightweight Monitoring System")
    
    # Initialize lightweight monitoring system
    safety_checker = LightweightSafetyChecker()
    system_monitor = LightweightSystemMonitor()
    
    cloudlog.info("Lightweight Monitoring System initialized successfully")