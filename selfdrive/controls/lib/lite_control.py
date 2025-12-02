#!/usr/bin/env python3
"""
Lightweight Control Systems for sunnypilot2
This module implements essential control improvements that are computationally efficient
for the Comma 3x hardware, focusing on high-impact, low-cost features.
"""

import numpy as np
from typing import Dict, Tuple
import time

# Import existing sunnypilot components
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX


class LightweightAdaptiveGainScheduler:
    """
    Lightweight adaptive gain scheduler that adjusts controller gains based on vehicle state.
    Computationally efficient version designed for Comma 3x constraints.
    Separates longitudinal and lateral gain scaling for safety.
    """
    def __init__(self, CP):
        self.CP = CP
        # Use simple filters with minimal computational overhead
        self.long_speed_filter = FirstOrderFilter(1.0, 1.0, 0.1)
        self.lat_speed_filter = FirstOrderFilter(1.0, 1.0, 0.1)
        self.long_thermal_filter = FirstOrderFilter(1.0, 1.0, 0.1)
        self.lat_thermal_filter = FirstOrderFilter(1.0, 1.0, 0.1)

    def get_adaptive_gains(self, v_ego: float, thermal_state: float) -> Dict:
        """
        Calculate lightweight adaptive gains based on vehicle speed and thermal state.
        Longitudinal and lateral gains are scaled independently for safety.
        """
        # Longitudinal speed-based gain adjustment
        # Lower gains at higher speeds for longitudinal control stability
        long_speed_factor = max(0.7, min(1.0, 1.0 - (v_ego - 10.0) / 40.0))  # Reduce beyond 10 m/s
        self.long_speed_filter.update(long_speed_factor)

        # Lateral speed-based gain adjustment
        # More aggressive reduction at higher speeds for lateral stability
        lat_speed_factor = max(0.5, min(1.0, 1.0 - (v_ego - 5.0) / 35.0))  # Reduce beyond 5 m/s
        self.lat_speed_filter.update(lat_speed_factor)

        # Thermal factors (different for longitudinal and lateral)
        # Longitudinal thermal factor (max 15% reduction to maintain braking capability)
        long_thermal_factor = max(0.85, 1.0 - thermal_state * 0.15)
        self.long_thermal_filter.update(long_thermal_factor)

        # Lateral thermal factor (max 10% reduction to maintain steering capability)
        lat_thermal_factor = max(0.9, 1.0 - thermal_state * 0.1)
        self.lat_thermal_filter.update(lat_thermal_factor)

        gains = {
            'longitudinal': {
                'kp': self.CP.longitudinalTuning.kp * self.long_speed_filter.x * self.long_thermal_filter.x,
                'ki': self.CP.longitudinalTuning.ki * self.long_speed_filter.x * self.long_thermal_filter.x,
                'kf': getattr(self.CP.longitudinalTuning, 'kf', 0.0) * self.long_speed_filter.x * self.long_thermal_filter.x
            },
            'lateral': {
                'kp': self.CP.lateralTuning.pid.kp * self.lat_speed_filter.x * self.lat_thermal_filter.x,
                'ki': self.CP.lateralTuning.pid.ki * self.lat_speed_filter.x * self.lat_thermal_filter.x,
                'kd': getattr(self.CP.lateralTuning.pid, 'kd', 0.0) * self.lat_speed_filter.x * self.lat_thermal_filter.x
            }
        }

        return gains


class LightweightComfortOptimizer:
    """
    Lightweight comfort optimization system to minimize jerk with minimal computation.
    """
    def __init__(self):
        self.prev_acceleration = 0.0
        self.prev_time = time.monotonic()
        self.comfort_jerk_limit = 1.5  # Base jerk limit (m/s^3)

    def optimize_for_comfort(self, desired_acceleration: float, v_ego: float) -> float:
        """
        Optimize acceleration command for passenger comfort using adaptive jerk limiting.
        No fixed loop rate assumption - calculates based on actual time delta.
        """
        current_time = time.monotonic()
        dt = current_time - self.prev_time if self.prev_time > 0 else 0.01  # Fallback to 100Hz

        # Adaptive jerk limit based on vehicle state
        adaptive_jerk_limit = self._calculate_adaptive_jerk_limit(v_ego)

        # Calculate desired jerk
        if dt > 0:
            desired_jerk = (desired_acceleration - self.prev_acceleration) / dt
        else:
            desired_jerk = 0.0

        # Apply adaptive jerk limit
        if abs(desired_jerk) > adaptive_jerk_limit:
            # Limit the change in acceleration to reduce jerk
            max_delta_a = adaptive_jerk_limit * dt
            new_acceleration = self.prev_acceleration + max_delta_a * np.sign(desired_jerk)
        else:
            new_acceleration = desired_acceleration

        # Apply vehicle limits
        new_acceleration = max(ACCEL_MIN, min(ACCEL_MAX, new_acceleration))

        # Update history
        self.prev_acceleration = new_acceleration
        self.prev_time = current_time

        return new_acceleration

    def _calculate_adaptive_jerk_limit(self, v_ego: float) -> float:
        """
        Calculate adaptive jerk limit based on vehicle speed and other factors.
        Lower jerk limits at higher speeds for safety and comfort.
        """
        # Base jerk limit decreases at higher speeds
        speed_factor = max(0.5, 1.0 - (v_ego / 50.0))  # Reduce limit as speed increases
        adaptive_limit = self.comfort_jerk_limit * speed_factor

        # Ensure minimum jerk limit
        return max(0.5, adaptive_limit)


# Example usage and testing
if __name__ == "__main__":
    from openpilot.common.params import Params
    from cereal import car
    cloudlog.info("Initializing Lightweight Control System")
    
    # Create mock parameters and car params for testing
    params = Params()
    CP = car.CarParams.new_message()
    
    # Initialize lightweight systems
    gain_scheduler = LightweightAdaptiveGainScheduler(CP)
    comfort_optimizer = LightweightComfortOptimizer()
    
    cloudlog.info("Lightweight Control System initialized successfully")