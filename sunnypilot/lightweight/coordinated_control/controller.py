"""
Simplified Coordinated Lateral-Longitudinal Controller for Sunnypilot2
Based on the optimized MPC control concepts but optimized for hardware constraints.

This module implements a simplified version of coordinated control that reduces
longitudinal acceleration during high lateral demand, mimicking human driving behavior.
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class ControlState:
    """Simple control state for coordinated control."""
    lateral_demand: float
    longitudinal_acceleration: float
    speed: float
    curvature: float


class SimplifiedCoordinatedController:
    """
    Simplified coordinated lateral-longitudinal controller optimized for Snapdragon 845.

    Uses a simple formula to reduce longitudinal acceleration when lateral demand is high,
    without the full MPC complexity of the original proposal.
    """

    def __init__(self,
                 lateral_tolerance: float = 0.15,
                 max_lat_accel: float = 2.5,
                 coordination_factor: float = 0.7):
        """
        Initialize the simplified coordinated controller.

        Args:
            lateral_tolerance: Threshold for considering lateral demand significant
            max_lat_accel: Maximum lateral acceleration allowed
            coordination_factor: Factor to reduce longitudinal accel based on lateral demand
        """
        self.lateral_tolerance = lateral_tolerance
        self.max_lat_accel = max_lat_accel
        self.coordination_factor = coordination_factor

    def adjust_acceleration(self,
                          base_acceleration: float,
                          lateral_demand: float,
                          speed: float,
                          curvature: float) -> float:
        """
        Adjust longitudinal acceleration based on lateral demand.

        Args:
            base_acceleration: Original longitudinal acceleration command
            lateral_demand: Current lateral acceleration demand
            speed: Current vehicle speed
            curvature: Current path curvature

        Returns:
            Adjusted longitudinal acceleration
        """
        # Calculate lateral acceleration based on speed and curvature
        lat_accel = speed * speed * abs(curvature)

        # Calculate coordination factor based on lateral demand
        if lat_accel > self.lateral_tolerance:
            # Reduce longitudinal acceleration proportionally to lateral demand
            reduction_factor = max(0.0, 1.0 - self.coordination_factor * (lat_accel / self.max_lat_accel))
            adjusted_accel = base_acceleration * reduction_factor  # Fixed: use base_acceleration

            # Check if sign changed (meaning coordination caused acceleration to switch to braking or vice versa)
            if np.sign(adjusted_accel) != np.sign(base_acceleration):
                # If original was braking (negative) and coordination wants to accelerate, maintain braking intent
                if base_acceleration <= 0:
                    # Keep braking but with reduced magnitude based on coordination
                    adjusted_accel = -max(abs(adjusted_accel), 0.3 * abs(base_acceleration))
                # If original was accelerating (positive) and coordination wants to brake,
                # allow the coordinated braking to take effect
                elif base_acceleration > 0:
                    # Allow braking but ensure it's not too gentle if the original intent was to brake
                    adjusted_accel = min(adjusted_accel, -0.3 * abs(base_acceleration))
            return adjusted_accel
        else:
            # No significant lateral demand, use original acceleration
            return base_acceleration

    def get_control_state(self,
                         acceleration: float,
                         lateral_demand: float,
                         speed: float,
                         curvature: float) -> ControlState:
        """
        Get the current control state.

        Args:
            acceleration: Current longitudinal acceleration
            lateral_demand: Current lateral acceleration demand
            speed: Current vehicle speed
            curvature: Current path curvature

        Returns:
            ControlState with current parameters
        """
        return ControlState(
            lateral_demand=lateral_demand,
            longitudinal_acceleration=acceleration,
            speed=speed,
            curvature=curvature
        )


class SafetyLimiter:
    """
    Safety limiter to ensure coordinated control doesn't compromise safety.
    """

    def __init__(self,
                 max_accel_for_braking: float = -1.0,
                 min_braking_for_curve: float = 0.5):
        """
        Initialize safety limiter.

        Args:
            max_accel_for_braking: Maximum acceleration allowed when significant braking needed
            min_braking_for_curve: Minimum braking in tight curves when needed
        """
        self.max_accel_for_braking = max_accel_for_braking
        self.min_braking_for_curve = min_braking_for_curve

    def apply_safety_limits(self,
                           adjusted_accel: float,
                           base_accel: float,
                           lat_accel: float,
                           speed: float,
                           lead_distance: float = float('inf'),
                           lead_velocity: float = 0.0) -> float:
        """
        Apply safety limits to the adjusted acceleration.

        Args:
            adjusted_accel: Acceleration after coordination adjustment
            base_accel: Original acceleration command
            lat_accel: Current lateral acceleration
            speed: Current vehicle speed
            lead_distance: Distance to lead vehicle
            lead_velocity: Velocity of lead vehicle

        Returns:
            Final acceleration after safety limits applied
        """
        final_accel = adjusted_accel

        # Check for potential collision with lead vehicle (TTC - Time To Collision)
        if lead_distance < float('inf') and lead_distance > 0:
            relative_velocity = speed - lead_velocity
            if relative_velocity > 0:
                ttc = lead_distance / relative_velocity  # Time to collision if no action taken
                if ttc < 2.0:  # Less than 2.0 seconds to collision - immediate danger
                    # Apply hard braking regardless of coordination
                    return self.max_accel_for_braking
                elif ttc < 3.0:  # Less than 3.0 seconds to collision - moderate danger
                    # Prioritize safety over coordination, but allow some adjustment
                    final_accel = min(adjusted_accel, self.max_accel_for_braking)

        # Check for dangerous combination of high speed and high lateral acceleration in curves
        if lat_accel > 3.5 and speed > 12.0:  # Very high lateral accel at moderate-high speed
            # Limit excessive acceleration in dangerous curve scenarios
            if final_accel > 0.5:
                final_accel = min(final_accel, 0.5)
            # Also ensure we don't prevent necessary braking in curves
            if final_accel < self.max_accel_for_braking and base_accel <= 0:
                final_accel = min(final_accel, self.max_accel_for_braking)

        return final_accel


def create_coordinated_controller() -> tuple[SimplifiedCoordinatedController, SafetyLimiter]:
    """
    Factory function to create coordinated controller and safety limiter
    optimized for Snapdragon 845 hardware constraints.
    """
    controller = SimplifiedCoordinatedController(
        lateral_tolerance=0.15,
        max_lat_accel=2.5,
        coordination_factor=0.7
    )

    safety_limiter = SafetyLimiter(
        max_accel_for_braking=-1.0,
        min_braking_for_curve=0.5
    )

    return controller, safety_limiter
