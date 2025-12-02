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
    """
    def __init__(self, CP):
        self.CP = CP
        # Use simple filters with minimal computational overhead
        self.speed_gain_filter = FirstOrderFilter(1.0, 1.0, 0.1)
        self.thermal_gain_filter = FirstOrderFilter(1.0, 1.0, 0.1)
        
    def get_adaptive_gains(self, v_ego: float, thermal_state: float) -> Dict:
        """
        Calculate lightweight adaptive gains based on vehicle speed and thermal state.
        """
        # Speed-based gain adjustment (simple and efficient)
        # Lower gains at higher speeds for stability
        speed_factor = max(0.5, min(1.0, 1.0 - (v_ego - 15.0) / 35.0))  # Reduce beyond 15 m/s
        self.speed_gain_filter.update(speed_factor)
        
        # Thermal factor (simple reduction when hot)
        thermal_factor = max(0.8, 1.0 - thermal_state * 0.2)  # Reduce up to 20% when hot
        self.thermal_gain_filter.update(thermal_factor)
        
        # Combined lightweight factor
        combined_factor = self.speed_gain_filter.x * self.thermal_gain_filter.x
        
        gains = {
            'longitudinal': {
                'kp': self.CP.longitudinalTuning.kp * combined_factor,
                'ki': self.CP.longitudinalTuning.ki * combined_factor,
                'kf': getattr(self.CP.longitudinalTuning, 'kf', 0.0) * combined_factor
            },
            'lateral': {
                'kp': self.CP.lateralTuning.pid.kp * combined_factor,
                'ki': self.CP.lateralTuning.pid.ki * combined_factor,
                'kd': getattr(self.CP.lateralTuning.pid, 'kd', 0.0) * combined_factor
            }
        }
        
        return gains


class LightweightComfortOptimizer:
    """
    Lightweight comfort optimization system to minimize jerk with minimal computation.
    """
    def __init__(self):
        self.prev_acceleration = 0.0
        self.comfort_jerk_limit = 1.5  # Reduced computational complexity
        
    def optimize_for_comfort(self, desired_acceleration: float, v_ego: float) -> float:
        """
        Optimize acceleration command for passenger comfort using simple jerk limiting.
        """
        # Calculate required jerk
        desired_jerk = (desired_acceleration - self.prev_acceleration) / 0.01  # 100Hz assumed
        
        # Apply simple jerk limit
        if abs(desired_jerk) > self.comfort_jerk_limit:
            # Limit the change in acceleration to reduce jerk
            max_delta_a = self.comfort_jerk_limit * 0.01
            new_acceleration = self.prev_acceleration + max_delta_a * np.sign(desired_jerk)
        else:
            new_acceleration = desired_acceleration
        
        # Apply vehicle limits
        new_acceleration = max(ACCEL_MIN, min(ACCEL_MAX, new_acceleration))
        
        # Update history
        self.prev_acceleration = new_acceleration
        
        return new_acceleration


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