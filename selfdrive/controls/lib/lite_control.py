#!/usr/bin/env python3
"""
Lightweight Control Systems for sunnypilot2
This module implements essential control improvements that are computationally efficient
for the Comma 3x hardware, focusing on high-impact, low-cost features.
"""

import numpy as np
from typing import Dict, Tuple, Optional
import time

# Import existing sunnypilot components
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX


def _interp_gain(x, xp, fp):
  """
  Linear interpolation function.
  x: The x-coordinate(s) at which to evaluate the interpolated values (v_ego).
  xp: The x-coordinates of the data points, must be increasing (breakpoint array).
  fp: The y-coordinates of the data points, same length as xp (gain array).
  """
  return np.interp(x, xp, fp)


class LightweightAdaptiveGainScheduler:
    """
    Lightweight adaptive gain scheduler that adjusts controller gains based on vehicle state.
    Computationally efficient version designed for Comma 3x constraints.
    Separates longitudinal and lateral gain scaling for safety.

    Architectural Note: This module depends on 'thermal_state' input, which is computed
    by the `LightweightSystemMonitor` in `lite_monitoring.py`. A robust integration
    strategy (e.g., via messaging queues, shared memory, or explicit object passing)
    must be established in the main control loop to ensure this data is reliably
    passed to this scheduler.
    """
    def __init__(self, CP):
        self.CP = CP
        # Use simple filters with minimal computational overhead
        self.long_speed_filter = FirstOrderFilter(0.1, 0.01, 0.01)  # Faster response
        self.lat_speed_filter = FirstOrderFilter(0.1, 0.01, 0.01)   # Faster response
        self.long_thermal_filter = FirstOrderFilter(0.2, 0.01, 0.01)  # Faster response
        self.lat_thermal_filter = FirstOrderFilter(0.2, 0.01, 0.01)   # Faster response

        # Vehicle-specific tuning parameters based on car model
        self.vehicle_tuning = self._get_vehicle_tuning(CP.carFingerprint)

    def _get_vehicle_tuning(self, car_model: str) -> Dict:
        """
        Get vehicle-specific tuning parameters based on the car model.
        """
        # Default tuning parameters
        tuning_params = {
            'long_speed_gain_factor': 30.0,  # Factor for longitudinal speed-based gain adjustment
            'lat_speed_gain_factor': 50.0,   # Factor for lateral speed-based gain adjustment
            'max_long_gain_multiplier': 1.5,  # Maximum longitudinal gain multiplier
            'max_lat_gain_multiplier': 1.3,   # Maximum lateral gain multiplier
            'min_long_gain_multiplier': 0.7,  # Minimum longitudinal gain multiplier
            'min_lat_gain_multiplier': 0.5    # Minimum lateral gain multiplier
        }

        # Customize tuning based on car model if specific parameters are available
        # IMPORTANT: These parameters are crucial for optimal performance and safety.
        # They MUST be tuned based on extensive real-world testing for each specific car model.
        # The commented-out examples serve as a template for structure, not as tuned values.
        car_specific_tuning = {
            'HYUNDAI_SONATA': {
                'long_speed_gain_factor': 25.0,
                'lat_speed_gain_factor': 45.0,
                'max_long_gain_multiplier': 1.4,
                'max_lat_gain_multiplier': 1.2,
                'min_long_gain_multiplier': 0.8,
                'min_lat_gain_multiplier': 0.6,
            },
            'TOYOTA_COROLLA': {
                'long_speed_gain_factor': 35.0,
                'lat_speed_gain_factor': 55.0,
                'max_long_gain_multiplier': 1.6,
                'max_lat_gain_multiplier': 1.4,
                'min_long_gain_multiplier': 0.9,
                'min_lat_gain_multiplier': 0.7,
            },
            # Add more car models with specific tuning as needed
            # These values would be tuned based on real-world testing
        }

        if car_model in car_specific_tuning:
            tuning_params.update(car_specific_tuning[car_model])

        return tuning_params

    def get_adaptive_gains(self, v_ego: float, thermal_state: float) -> Dict:
        """
        Calculate lightweight adaptive gains based on vehicle speed and thermal state.
        Longitudinal and lateral gains are scaled independently for safety.
        """
        # Longitudinal speed-based gain adjustment using vehicle-specific parameters
        # Higher gains at higher speeds for better control authority at highway speeds
        long_speed_factor = max(
            self.vehicle_tuning['min_long_gain_multiplier'],
            min(
                self.vehicle_tuning['max_long_gain_multiplier'],
                1.0 + (v_ego / self.vehicle_tuning['long_speed_gain_factor'])
            )
        )
        self.long_speed_filter.update(long_speed_factor)

        # Lateral speed-based gain adjustment using vehicle-specific parameters
        # Moderate increase at higher speeds for better lateral control
        lat_speed_factor = max(
            self.vehicle_tuning['min_lat_gain_multiplier'],
            min(
                self.vehicle_tuning['max_lat_gain_multiplier'],
                1.0 + (v_ego / self.vehicle_tuning['lat_speed_gain_factor'])
            )
        )
        self.lat_speed_filter.update(lat_speed_factor)

        # Thermal factors (different for longitudinal and lateral)
        # Longitudinal thermal factor (max 15% reduction to maintain braking capability)
        long_thermal_factor = max(0.85, 1.0 - thermal_state * 0.15)
        self.long_thermal_filter.update(long_thermal_factor)

        # Lateral thermal factor (max 10% reduction to maintain steering capability)
        lat_thermal_factor = max(0.9, 1.0 - thermal_state * 0.1)
        cloudlog.debug(f"GainScheduler: v_ego={v_ego:.2f} m/s, thermal_state={thermal_state:.2f}")
        cloudlog.debug(f"GainScheduler: Long Speed Factor={long_speed_factor:.2f}, Filtered Long Speed Factor={self.long_speed_filter.x:.2f}")
        cloudlog.debug(f"GainScheduler: Lat Speed Factor={lat_speed_factor:.2f}, Filtered Lat Speed Factor={self.lat_speed_filter.x:.2f}")
        cloudlog.debug(f"GainScheduler: Long Thermal Factor={long_thermal_factor:.2f}, Filtered Long Thermal Factor={self.long_thermal_filter.x:.2f}")
        cloudlog.debug(f"GainScheduler: Lat Thermal Factor={lat_thermal_factor:.2f}, Filtered Lat Thermal Factor={self.lat_thermal_filter.x:.2f}")

        # Calculate base gains from tuning tables, using interpolation
        # For longitudinal:
        base_long_kp = self._get_base_gain(v_ego, self.CP.longitudinalTuning.kpBP, self.CP.longitudinalTuning.kpV, 1.0, "longitudinal kpV")
        base_long_ki = self._get_base_gain(v_ego, self.CP.longitudinalTuning.kiBP, self.CP.longitudinalTuning.kiV, 0.1, "longitudinal kiV")
        base_long_kf = getattr(self.CP.longitudinalTuning, 'kf', 0.0)

        # For lateral:
        base_lat_kp = self._get_base_gain(v_ego, self.CP.lateralTuning.pid.kpBP, self.CP.lateralTuning.pid.kpV, 0.5, "lateral kpV")
        base_lat_ki = self._get_base_gain(v_ego, self.CP.lateralTuning.pid.kiBP, self.CP.lateralTuning.pid.kiV, 0.05, "lateral kiV")
        base_lat_kd = 0.0
        # Check if kdV exists and if a corresponding kdBP exists for interpolation
        if hasattr(self.CP.lateralTuning.pid, 'kdV') and self.CP.lateralTuning.pid.kdV:
            base_lat_kd = self._get_base_gain(v_ego, getattr(self.CP.lateralTuning.pid, 'kdBP', None), self.CP.lateralTuning.pid.kdV, 0.0, "lateral kdV")
        base_lat_kf = getattr(self.CP.lateralTuning.pid, 'kf', 0.0)

        gains = {
            'longitudinal': {
                'kp': base_long_kp * self.long_speed_filter.x * self.long_thermal_filter.x,
                'ki': base_long_ki * self.long_speed_filter.x * self.long_thermal_filter.x,
                'kf': base_long_kf * self.long_speed_filter.x * self.long_thermal_filter.x
            },
            'lateral': {
                'kp': base_lat_kp * self.lat_speed_filter.x * self.lat_thermal_filter.x,
                'ki': base_lat_ki * self.lat_speed_filter.x * self.lat_thermal_filter.x,
                'kd': base_lat_kd * self.lat_speed_filter.x * self.lat_thermal_filter.x,  # Fixed: Using proper kd value instead of kf
                'kf': base_lat_kf * self.lat_speed_filter.x * self.lat_thermal_filter.x
            }
        }

        cloudlog.debug(f"GainScheduler: Final Longitudinal Gains: kp={gains['longitudinal']['kp']:.3f}, ki={gains['longitudinal']['ki']:.3f}, kf={gains['longitudinal']['kf']:.3f}")
        cloudlog.debug(f"GainScheduler: Final Lateral Gains: kp={gains['lateral']['kp']:.3f}, ki={gains['lateral']['ki']:.3f}, kd={gains['lateral']['kd']:.3f}, kf={gains['lateral']['kf']:.3f}")

        return gains

    def _get_base_gain(self, v_ego: float, bp_array: Optional[list], gain_array: list, default_gain: float, gain_name: str) -> float:
        """
        Helper method to safely get a gain value, using linear interpolation if breakpoints are provided.
        If breakpoint or gain arrays are missing/invalid for interpolation, it falls back to a single value (gain_array[0])
        or the default_gain.
        """
        # Check if interpolation is possible
        if (bp_array is not None and
                hasattr(bp_array, '__getitem__') and len(bp_array) > 0 and
                hasattr(gain_array, '__getitem__') and len(gain_array) > 0 and
                len(bp_array) == len(gain_array)):
            return _interp_gain(v_ego, bp_array, gain_array)
        else:
            # Fallback to single value or default if interpolation not possible
            if hasattr(gain_array, '__getitem__') and len(gain_array) > 0:
                cloudlog.warning(f"No valid BP array provided for {gain_name}, using first value: {gain_array[0]}")
                return gain_array[0]
            else:
                cloudlog.warning(f"Missing or invalid {gain_name} parameter (no valid array), using default value: {default_gain}")
                return default_gain


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
        # Base jerk limit decreases at higher speeds (0 m/s to 30 m/s range)
        # Reduce limit from 1.5 to 0.8 as speed increases from 0 to 30 m/s
        speed_factor = max(0.5, 1.0 - (v_ego / 30.0))  # Reduce limit as speed increases up to 30 m/s
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

    # Example of how to use the system with a thermal state from monitoring system
    v_ego = 20.0  # Current speed in m/s
    thermal_state = 0.3  # Example thermal state from monitoring system
    gains = gain_scheduler.get_adaptive_gains(v_ego, thermal_state)

    cloudlog.info("Lightweight Control System initialized successfully")