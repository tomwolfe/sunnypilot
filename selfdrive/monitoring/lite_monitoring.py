#!/usr/bin/env python3
"""
Lightweight Monitoring System for sunnypilot2
This module implements essential safety checks with minimal computational overhead
designed for Comma 3x hardware constraints.
"""

import numpy as np
from typing import List, cast

import time

# Import existing sunnypilot components
from openpilot.common.swaglog import cloudlog

from cereal import car


class LightweightSafetyChecker:
    """
    Lightweight safety validation system with minimal computational overhead.
    """
    def __init__(self, CP=None):
        self.CP = CP
        # Vehicle-specific safety thresholds based on car model and parameters
        self.safety_thresholds = self._get_vehicle_specific_thresholds(CP) if CP else {
            'max_long_accel': 3.0,     # Maximum longitudinal acceleration (m/s^2)
            'max_lat_accel': 2.5,      # Maximum lateral acceleration (m/s^2)
            'max_steering_rate': 1.0,  # Maximum steering rate (rad/s). Default increased from 0.5.
                                       # This value may require tuning per vehicle to avoid unnecessary disengagements.
            'min_time_gap': 2.0,       # Minimum safe time gap (s) for active intervention
        }
        # Initialize tracking variables for steering rate calculation
        self._prev_steer = 0.0
        self._prev_time = time.monotonic()

    def _get_vehicle_specific_thresholds(self, CP):
        """
        Get vehicle-specific safety thresholds based on CarParams for proper adaptation.
        """
        base_thresholds = {
            'max_long_accel': 3.0,     # Maximum longitudinal acceleration (m/s^2)
            'max_lat_accel': 2.5,      # Maximum lateral acceleration (m/s^2)
            'max_steering_rate': 0.3,  # Default maximum steering rate (rad/s)
            'max_steering_angle': 0.5, # Default maximum steering angle (rad)
            'min_time_gap': 2.0,       # Minimum safe time gap (s) for active intervention
        }

        # Adjust thresholds based on specific car parameters if available
        if CP:
            # Use CP.steerMax for max_steering_angle, which is the physical steering angle limit in radians.
            # A small buffer is added for safety to prevent hitting the absolute limit.
            if CP.steerMax:
                base_thresholds['max_steering_angle'] = CP.steerMax * 0.95 # 95% of max steer to allow some buffer

            # Dynamically adjust max_steering_rate based on steerMax.
            # Cars with higher steerMax can typically handle faster steering rates.
            # Using a scaling factor, but capping it to remain conservative.
            if CP.steerMax:
                # A simple linear scaling, but ensuring it doesn't go too high
                # Start with a base of 0.3 and add a scaled amount based on steerMax
                scaled_rate = 0.3 + (CP.steerMax / 10.0) # Example: steerMax 1.0 -> 0.4 rad/s, steerMax 2.0 -> 0.5 rad/s
                cloudlog.debug(f"SafetyChecker: CP.steerMax={CP.steerMax:.2f}, calculated scaled_rate={scaled_rate:.2f}")
                # IMPORTANT: This heuristic needs validation with real-world data across different vehicle models
                base_thresholds['max_steering_rate'] = min(scaled_rate, 0.8) # Cap at 0.8 rad/s, requires further tuning for safety
                cloudlog.debug(f"SafetyChecker: Final max_steering_rate set to {base_thresholds['max_steering_rate']:.2f} rad/s")

        return base_thresholds

    def validate_outputs(self, actuators, car_state, radar_state) -> dict:
        """
        Lightweight validation of control outputs for safety.
        Returns a detailed report instead of just True/False.
        """
        safety_report = {
            'safe': True,
            'violations': cast(List[str], []),
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
        if hasattr(actuators, 'steer'):
            current_time = time.monotonic()
            # Calculate the steering rate using current and previous values
            if hasattr(self, '_prev_time') and self._prev_time > 0:
                time_delta = current_time - self._prev_time
                steering_rate = abs(actuators.steer - self._prev_steer) / max(time_delta, 0.01)  # Use a small dt to avoid division by zero
                cloudlog.debug(f"SafetyChecker: Steering rate={steering_rate:.2f} rad/s, Max rate={self.safety_thresholds['max_steering_rate']:.2f} rad/s")
                if steering_rate > self.safety_thresholds['max_steering_rate']:
                    safety_report['safe'] = False
                    safety_report['violations'].append('steering_rate_limit_exceeded')
                    safety_report['recommended_action'] = 'disengage'
                    cloudlog.warning(f"SafetyChecker: Steering rate limit exceeded! "
                                  f"rate={{steering_rate:.2f}}, "
                                  f"limit={{self.safety_thresholds['max_steering_rate']:.2f}}, "
                                  f"vEgo={{car_state.vEgo:.2f}}")


            # Update current values immediately after calculation to prevent race condition
            self._prev_steer = actuators.steer
            self._prev_time = current_time

        # Enhanced forward collision check - handles all lead vehicle scenarios
        if (hasattr(radar_state, 'leadOne') and
            radar_state.leadOne.status and
            car_state.vEgo > 0.1):

            # Calculate relative speed: positive means approaching, negative means moving away or lead is faster
            relative_speed = car_state.vEgo - radar_state.leadOne.vRel

            # Only calculate TTC if vehicles are approaching each other (relative_speed > 0)
            if relative_speed > 0.1:
                ttc = radar_state.leadOne.dRel / relative_speed
            else:
                # If relative speed is 0 or negative, vehicles are not approaching (lead is moving away or faster)
                # Set TTC to infinity since there's no immediate collision risk
                ttc = float('inf')

            # Check for potential collision based on TTC or distance
            min_safe_distance_calculated = car_state.vEgo * self.safety_thresholds['min_time_gap']
            if ttc < self.safety_thresholds['min_time_gap'] or radar_state.leadOne.dRel < min_safe_distance_calculated:
                # This is a safety-critical situation regardless of current acceleration
                safety_report['safe'] = False
                safety_report['violations'].append('forward_collision_imminent')
                safety_report['recommended_action'] = 'decelerate'
        # Determine recommended_action based on violations, with disengage having priority
        if not safety_report['safe']:
            if 'steering_angle_limit_exceeded' in safety_report['violations'] or \
               'steering_rate_limit_exceeded' in safety_report['violations']:
                safety_report['recommended_action'] = 'disengage'
                        elif 'long_accel_limit_exceeded' in safety_report['violations'] or \
                             'lat_accel_limit_exceeded' in safety_report['violations'] or \
                             'forward_collision_imminent' in safety_report['violations']:
                                             safety_report['recommended_action'] = 'decelerate'
                                    return safety_report    def trigger_fail_safe(self, safety_report: dict, car_state) -> dict:
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
                v_ego = car_state.vEgo if hasattr(car_state, 'vEgo') else 0.0

                # Use CP.stopAccel if available, otherwise fall back to hardcoded values
                if self.CP and hasattr(self.CP, 'stopAccel') and isinstance(self.CP.stopAccel, (int, float)):
                    calculated_deceleration = self.CP.stopAccel
                    cloudlog.debug(f"SafetyChecker: Using CP.stopAccel for deceleration: {calculated_deceleration:.2f} m/s^2")
                else:
                    v_points = [0.0, 10.0, 30.0]  # Speeds in m/s
                    decel_points = [-2.0, -3.0, -2.5] # Deceleration values in m/s^2
                    calculated_deceleration = float(np.interp(v_ego, v_points, decel_points))
                    cloudlog.debug(f"SafetyChecker: Using default interp for deceleration: {calculated_deceleration:.2f} m/s^2")

                return {
                    'acceleration': calculated_deceleration,
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

    def check_system_health(self, device_state, car_state) -> dict:
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
            ram_usage_percent = device_state.memoryUsagePercent
            # `min_ram_free` is a fraction (e.g., 0.1 for 10% free).
            # Convert to max allowed usage percentage for easier comparison.
            max_ram_usage_percent = (1.0 - self.health_thresholds['min_ram_free']) * 100.0
            if ram_usage_percent > max_ram_usage_percent:
                health_report['ram_low'] = True

        # Additional checks for CAN bus, disk, etc. if available
        # A more robust check would verify that the specific messages needed for control
        # (e.g., carState, radarState) are arriving at their expected frequency
        if hasattr(device_state, 'canMonoTimes'):
            if not device_state.canMonoTimes: # If list is empty, CAN bus is not ok
                health_report['can_bus_ok'] = False
            else:
                current_mono_time = time.monotonic()
                recent_can_message_found = False
                # Check if recent CAN messages have been received within expected time window (2 seconds)
                for can_time_ns in device_state.canMonoTimes: # canMonoTimes are in nanoseconds
                    if (current_mono_time - (can_time_ns * 1e-9)) < 2.0: # Check if message is within last 2 seconds
                        recent_can_message_found = True
                        break
                if not recent_can_message_found:
                    health_report['can_bus_ok'] = False

        # Check disk space if available
        if hasattr(device_state, 'freeSpacePercent') and device_state.freeSpacePercent:
            if device_state.freeSpacePercent < 5:  # Less than 5% free (assuming value is 0-100%)
                health_report['disk_space_ok'] = False

        # Log periodically to reduce overhead
        current_time = time.monotonic()
        if current_time - self.last_log_time > self.log_interval:
            # Only log basic info periodically
            cpu_temp = max(device_state.cpuTempC) if hasattr(device_state, 'cpuTempC') and device_state.cpuTempC else 'N/A'
            gpu_temp = max(device_state.gpuTempC) if hasattr(device_state, 'gpuTempC') and device_state.gpuTempC else 'N/A'
            cpu_usage_val = device_state.cpuUsagePercent if hasattr(device_state, 'cpuUsagePercent') and device_state.cpuUsagePercent else 'N/A'
            ram_usage_val = device_state.memoryUsagePercent if hasattr(device_state, 'memoryUsagePercent') and device_state.memoryUsagePercent else 'N/A'

            cloudlog.debug(f"System Health: CPU {cpu_temp}, GPU {gpu_temp}, CPU% {cpu_usage_val}, RAM% {ram_usage_val}")
            self.last_log_time = current_time

        return health_report

    def calculate_thermal_state(self, device_state) -> float:
        """
        Calculate normalized thermal state (0.0 to 1.0) based on system temperature readings.
        0.0 = cold/normal operation, 1.0 = critical thermal state
        """
        # Get CPU and GPU temperatures if available
        cpu_temp = 0.0
        gpu_temp = 0.0

        if hasattr(device_state, 'cpuTempC') and device_state.cpuTempC:
            cpu_temps = device_state.cpuTempC if isinstance(device_state.cpuTempC, list) else [device_state.cpuTempC]
            cpu_temp = max(cpu_temps) if cpu_temps else 0.0

        if hasattr(device_state, 'gpuTempC') and device_state.gpuTempC:
            gpu_temps = device_state.gpuTempC if isinstance(device_state.gpuTempC, list) else [device_state.gpuTempC]
            gpu_temp = max(gpu_temps) if gpu_temps else 0.0

        # Calculate normalized thermal state based on thresholds (0.0 to 1.0)
        # Using a piecewise linear scale:
        # Below 40°C = 0.0 (cool/normal)
        # 40°C to 70°C = 0.0 to 0.5 (linear increase)
        # 70°C to 90°C = 0.5 to 1.0 (more aggressive linear increase)
        temp_points = [0.0, 40.0, 70.0, 90.0] # Temperatures in Celsius
        thermal_state_values = [0.0, 0.0, 0.5, 1.0] # Normalized thermal state values
        thermal_state = float(np.interp(max(cpu_temp, gpu_temp), temp_points, thermal_state_values))

        return thermal_state


# Example usage and testing
if __name__ == "__main__":
    from cereal import car
    cloudlog.info("Initializing Lightweight Monitoring System")

    # Create mock CarParams for testing
    CP = car.CarParams.new_message()

    # Initialize lightweight monitoring system with CarParams
    safety_checker = LightweightSafetyChecker(CP)
    system_monitor = LightweightSystemMonitor()

    cloudlog.info("Lightweight Monitoring System initialized successfully")
