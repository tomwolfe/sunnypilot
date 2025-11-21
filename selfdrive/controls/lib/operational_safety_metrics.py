#!/usr/bin/env python3
"""
Operational safety metrics for sunnypilot autonomous driving system
Defines measurable safety requirements and provides verification tests
"""

from typing import Dict, Tuple, Optional
import numpy as np
from openpilot.common.swaglog import cloudlog


class OperationalSafetyMetrics:
    """
    Defines and verifies operational safety metrics for autonomous driving
    Based on measurable safety requirements like minimum following distance,
    maximum lateral acceleration during high-risk scenarios, etc.
    """

    def __init__(self):
        # Operational safety requirements
        self.MIN_FOLLOWING_DISTANCE_METERS = 30.0  # Minimum following distance under all conditions
        self.MAX_FOLLOWING_TIME_SECONDS = 2.0     # Maximum following time rule (NHTSA recommendation)
        self.MIN_FOLLOWING_TIME_SECONDS = 1.5     # Minimum following time for safety
        self.MAX_LATERAL_ACCEL_HIGH_RISK = 1.5    # Maximum lateral acceleration during high-risk scenarios
        self.MAX_LONGITUDINAL_JERK = 3.0          # Maximum longitudinal jerk for safety
        self.MAX_LATERAL_JERK = 2.0               # Maximum lateral jerk for safety
        self.MAX_CURVATURE_RATE = 0.1             # Maximum curvature rate change per second

    def verify_following_distance(self, current_distance: float, 
                                lead_velocity: float, 
                                ego_velocity: float) -> Tuple[bool, Dict[str, float]]:
        """
        Verify that following distance meets operational safety requirements
        :param current_distance: Current distance to lead vehicle (meters)
        :param lead_velocity: Lead vehicle velocity (m/s)
        :param ego_velocity: Ego vehicle velocity (m/s)
        :return: Tuple of (is_safe, details)
        """
        # Check minimum distance requirement
        min_distance_safe = current_distance >= self.MIN_FOLLOWING_DISTANCE_METERS

        # Calculate time to collision if both vehicles brake suddenly
        # For now, just calculate the time-based following distance
        if ego_velocity > 0:
            following_time = current_distance / ego_velocity if ego_velocity > 0 else float('inf')
            time_based_safe = (following_time >= self.MIN_FOLLOWING_TIME_SECONDS and 
                             following_time <= self.MAX_FOLLOWING_TIME_SECONDS)
        else:
            time_based_safe = True  # Stationary vehicle

        # Calculate safe stopping distance
        # Assuming 0.5s reaction time + braking distance
        reaction_time = 0.5  # seconds
        max_deceleration = -3.5  # m/s^2
        safe_stopping_distance = (ego_velocity * reaction_time + 
                                 (ego_velocity ** 2) / (2 * abs(max_deceleration)))

        stopping_distance_safe = current_distance >= safe_stopping_distance

        is_safe = min_distance_safe and time_based_safe and stopping_distance_safe
        details = {
            'current_distance': current_distance,
            'following_time': following_time if ego_velocity > 0 else float('inf'),
            'min_distance_safe': min_distance_safe,
            'time_based_safe': time_based_safe,
            'stopping_distance_safe': stopping_distance_safe,
            'required_stopping_distance': safe_stopping_distance
        }

        return is_safe, details

    def verify_lateral_acceleration(self, lateral_acceleration: float, 
                                  environmental_risk: float) -> Tuple[bool, Dict[str, float]]:
        """
        Verify lateral acceleration is within operational safety limits
        :param lateral_acceleration: Current lateral acceleration (m/s^2)
        :param environmental_risk: Current environmental risk (0.0-1.0)
        :return: Tuple of (is_safe, details)
        """
        # Maximum lateral acceleration depends on environmental risk
        if environmental_risk > 0.7:  # High risk
            max_lat_accel = self.MAX_LATERAL_ACCEL_HIGH_RISK
        elif environmental_risk > 0.4:  # Medium risk
            max_lat_accel = self.MAX_LATERAL_ACCEL_HIGH_RISK * 1.2  # Slightly higher for medium risk
        else:  # Low risk
            max_lat_accel = self.MAX_LATERAL_ACCEL_HIGH_RISK * 1.5  # Higher for low risk

        is_safe = abs(lateral_acceleration) <= max_lat_accel
        details = {
            'lateral_acceleration': lateral_acceleration,
            'environmental_risk': environmental_risk,
            'max_allowed_lateral_accel': max_lat_accel,
            'is_safe': is_safe
        }

        return is_safe, details

    def verify_longitudinal_jerk(self, current_accel: float, 
                               prev_accel: float, 
                               dt: float) -> Tuple[bool, Dict[str, float]]:
        """
        Verify longitudinal jerk is within operational safety limits
        :param current_accel: Current longitudinal acceleration (m/s^2)
        :param prev_accel: Previous longitudinal acceleration (m/s^2)
        :param dt: Time delta (seconds)
        :return: Tuple of (is_safe, details)
        """
        if dt <= 0:
            return True, {'jerk': 0.0, 'is_safe': True, 'message': 'Zero time delta'}

        jerk = abs(current_accel - prev_accel) / dt
        is_safe = jerk <= self.MAX_LONGITUDINAL_JERK
        details = {
            'current_accel': current_accel,
            'prev_accel': prev_accel,
            'dt': dt,
            'jerk': jerk,
            'max_jerk': self.MAX_LONGITUDINAL_JERK,
            'is_safe': is_safe
        }

        return is_safe, details

    def verify_lateral_jerk(self, current_lat_accel: float, 
                          prev_lat_accel: float, 
                          dt: float) -> Tuple[bool, Dict[str, float]]:
        """
        Verify lateral jerk is within operational safety limits
        :param current_lat_accel: Current lateral acceleration (m/s^2)
        :param prev_lat_accel: Previous lateral acceleration (m/s^2)
        :param dt: Time delta (seconds)
        :return: Tuple of (is_safe, details)
        """
        if dt <= 0:
            return True, {'lateral_jerk': 0.0, 'is_safe': True, 'message': 'Zero time delta'}

        lateral_jerk = abs(current_lat_accel - prev_lat_accel) / dt
        is_safe = lateral_jerk <= self.MAX_LATERAL_JERK
        details = {
            'current_lat_accel': current_lat_accel,
            'prev_lat_accel': prev_lat_accel,
            'dt': dt,
            'lateral_jerk': lateral_jerk,
            'max_lateral_jerk': self.MAX_LATERAL_JERK,
            'is_safe': is_safe
        }

        return is_safe, details

    def verify_curvature_rate(self, current_curvature: float, 
                            prev_curvature: float, 
                            dt: float, 
                            v_ego: float) -> Tuple[bool, Dict[str, float]]:
        """
        Verify curvature rate is within operational safety limits
        :param current_curvature: Current curvature (1/m)
        :param prev_curvature: Previous curvature (1/m)
        :param dt: Time delta (seconds)
        :param v_ego: Current vehicle speed (m/s)
        :return: Tuple of (is_safe, details)
        """
        if dt <= 0:
            return True, {'curvature_rate': 0.0, 'is_safe': True, 'message': 'Zero time delta'}

        curvature_rate = abs(current_curvature - prev_curvature) / dt
        is_safe = curvature_rate <= self.MAX_CURVATURE_RATE

        # Also consider that at higher speeds, even moderate curvature rates can result in high lateral accelerations
        # Lateral accel = v^2 * curvature
        max_possible_lat_accel = (v_ego ** 2) * current_curvature if abs(current_curvature) > abs(prev_curvature) else (v_ego ** 2) * prev_curvature

        details = {
            'current_curvature': current_curvature,
            'prev_curvature': prev_curvature,
            'dt': dt,
            'curvature_rate': curvature_rate,
            'max_curvature_rate': self.MAX_CURVATURE_RATE,
            'vehicle_speed': v_ego,
            'max_possible_lat_accel': abs(max_possible_lat_accel),
            'is_safe': is_safe
        }

        return is_safe, details

    def verify_all_operational_metrics(self, following_distance: float, 
                                     lead_velocity: float, 
                                     ego_velocity: float,
                                     lateral_accel: float,
                                     environmental_risk: float,
                                     current_accel: float,
                                     prev_accel: float,
                                     current_lat_accel: float,
                                     prev_lat_accel: float,
                                     current_curvature: float,
                                     prev_curvature: float,
                                     dt: float) -> Dict[str, Tuple[bool, Dict[str, float]]]:
        """
        Verify all operational safety metrics at once
        :return: Dictionary of all verification results
        """
        results = {}

        results['following_distance'] = self.verify_following_distance(
            following_distance, lead_velocity, ego_velocity
        )

        results['lateral_acceleration'] = self.verify_lateral_acceleration(
            lateral_accel, environmental_risk
        )

        results['longitudinal_jerk'] = self.verify_longitudinal_jerk(
            current_accel, prev_accel, dt
        )

        results['lateral_jerk'] = self.verify_lateral_jerk(
            current_lat_accel, prev_lat_accel, dt
        )

        results['curvature_rate'] = self.verify_curvature_rate(
            current_curvature, prev_curvature, dt, ego_velocity
        )

        return results

    def get_operational_safety_score(self, metrics_results: Dict[str, Tuple[bool, Dict[str, float]]]) -> float:
        """
        Calculate an overall operational safety score (0.0-1.0) based on all metrics
        :param metrics_results: Results from verify_all_operational_metrics
        :return: Safety score where 1.0 is completely safe and 0.0 is completely unsafe
        """
        if not metrics_results:
            return 0.0

        # Count safe metrics
        safe_count = sum(1 for is_safe, _ in metrics_results.values() if is_safe)
        total_count = len(metrics_results)

        if total_count == 0:
            return 1.0  # No metrics to check, assume safe

        return safe_count / total_count


def run_operational_safety_tests():
    """
    Run verification tests for operational safety metrics
    """
    print("Running operational safety verification tests...")
    metrics = OperationalSafetyMetrics()

    # Test following distance verification
    safe, details = metrics.verify_following_distance(50.0, 20.0, 25.0)
    print(f"Following distance test (50m at 25m/s): {'PASS' if safe else 'FAIL'} - {details}")

    # Test lateral acceleration verification with high environmental risk
    safe, details = metrics.verify_lateral_acceleration(1.0, 0.8)
    print(f"Lateral acceleration test (1.0m/s², risk 0.8): {'PASS' if safe else 'FAIL'} - {details}")

    # Test longitudinal jerk verification
    safe, details = metrics.verify_longitudinal_jerk(2.0, 1.0, 0.05)
    print(f"Longitudinal jerk test (2.0 to 1.0 m/s² in 0.05s): {'PASS' if safe else 'FAIL'} - {details}")

    # Test lateral jerk verification
    safe, details = metrics.verify_lateral_jerk(0.5, 0.2, 0.05)
    print(f"Lateral jerk test (0.5 to 0.2 m/s² in 0.05s): {'PASS' if safe else 'FAIL'} - {details}")

    # Test curvature rate verification
    safe, details = metrics.verify_curvature_rate(0.005, 0.002, 0.05, 20.0)
    print(f"Curvature rate test (0.002 to 0.005 in 0.05s at 20m/s): {'PASS' if safe else 'FAIL'} - {details}")

    print("Operational safety tests completed!")


if __name__ == "__main__":
    run_operational_safety_tests()