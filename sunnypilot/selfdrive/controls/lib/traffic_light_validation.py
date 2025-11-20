"""
Enhanced Traffic Sign Validation for Sunnypilot
Comprehensive implementation for traffic sign detection and validation with sensor fusion
"""

import numpy as np
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass
import time
from enum import Enum
from collections import defaultdict
import math


class TrafficSignType(Enum):
    """Types of traffic signs and signals"""
    TRAFFIC_LIGHT_RED = "traffic_light_red"
    TRAFFIC_LIGHT_YELLOW = "traffic_light_yellow"
    TRAFFIC_LIGHT_GREEN = "traffic_light_green"
    STOP_SIGN = "stop_sign"
    YIELD_SIGN = "yield_sign"
    SPEED_LIMIT = "speed_limit"
    PEDESTRIAN_CROSSING = "pedestrian_crossing"
    SCHOOL_ZONE = "school_zone"


@dataclass
class TrafficSignData:
    """Enhanced data structure for detected traffic sign"""
    sign_type: TrafficSignType
    distance: float  # Distance to sign in meters
    confidence: float  # Detection confidence (0-1)
    position: np.ndarray  # Position [x, y, z]
    timestamp: float = 0.0
    additional_data: Dict[str, Any] = None
    signal_state: str = "unknown"  # For traffic lights: red, yellow, green, unknown
    sign_value: float = 0.0  # For speed limit signs, etc.
    heading: float = 0.0  # Heading of sign relative to vehicle (in radians)


class TrafficValidator:
    """Enhanced traffic validation system with sensor fusion and predictive validation"""

    def __init__(self):
        self.traffic_signs = []
        self.max_signs = 50  # Increased to handle multiple signs in route
        self.validity_threshold = 5.0  # Extended threshold for better context
        self.min_confidence = 0.7  # Minimum confidence for valid detection
        self.stopping_distance_buffer = 5.0  # Extra distance buffer for safety

        # Track validation history for adaptive learning
        self.validation_history = defaultdict(list)

        # Traffic light state prediction
        self.traffic_light_states = {}

    def add_traffic_sign_data(self, sign_data: TrafficSignData):
        """Add detected traffic sign data with validation"""
        if sign_data.confidence < self.min_confidence:
            return  # Filter out low-confidence detections

        current_time = time.time()

        # Filter out expired signs
        self.traffic_signs = [
            sign for sign in self.traffic_signs
            if current_time - sign.timestamp < self.validity_threshold
        ]

        # Limit number of signs to prevent memory bloat
        if len(self.traffic_signs) >= self.max_signs:
            self.traffic_signs = self.traffic_signs[-(self.max_signs - 1):]

        sign_data.timestamp = current_time
        self.traffic_signs.append(sign_data)

    def validate_traffic_lights(self, position: np.ndarray, velocity: float,
                              heading: float = 0.0,
                              desired_speed: float = 0.0) -> Tuple[bool, List[str], Dict[str, Any]]:
        """Validate traffic light compliance with enhanced logic"""
        violations = []
        recommendations = {}

        # Find traffic lights in range
        traffic_lights = [
            sign for sign in self.traffic_signs
            if sign.sign_type in [TrafficSignType.TRAFFIC_LIGHT_RED,
                                TrafficSignType.TRAFFIC_LIGHT_YELLOW,
                                TrafficSignType.TRAFFIC_LIGHT_GREEN]
        ]

        if not traffic_lights:
            return True, [], {}

        # Get closest traffic light
        closest_light = min(
            traffic_lights,
            key=lambda x: np.linalg.norm(position - x.position)
        )

        distance_to_light = np.linalg.norm(position - closest_light.position)

        # Calculate stopping distance based on current velocity and road conditions
        stopping_distance = self._calculate_stopping_distance(velocity)

        # Check traffic light violations with enhanced logic
        if closest_light.sign_type == TrafficSignType.TRAFFIC_LIGHT_RED:
            safe_approach = self._is_safe_approach_red_light(
                velocity, distance_to_light, stopping_distance
            )
            if not safe_approach:
                violations.append("APPROACHING_RED_LIGHT_TOO_FAST")
                recommendations['recommended_action'] = 'STOP_IMMEDIATELY'

        elif closest_light.sign_type == TrafficSignType.TRAFFIC_LIGHT_YELLOW:
            if velocity > desired_speed * 0.7 and distance_to_light < stopping_distance * 1.2:
                # Yellow light: if we can't safely stop, proceed, but if we can, slow down
                violations.append("APPROACHING_YELLOW_LIGHT_UNSAFELY")

        return len(violations) == 0, violations, recommendations

    def validate_stop_signs(self, position: np.ndarray, velocity: float,
                          heading: float = 0.0) -> Tuple[bool, List[str], Dict[str, Any]]:
        """Validate stop sign compliance with enhanced logic"""
        violations = []
        recommendations = {}

        # Find stop signs
        stop_signs = [
            sign for sign in self.traffic_signs
            if sign.sign_type in [TrafficSignType.STOP_SIGN, TrafficSignType.YIELD_SIGN]
        ]

        if not stop_signs:
            return True, [], {}

        # Get closest stop sign
        closest_sign = min(
            stop_signs,
            key=lambda x: np.linalg.norm(position - x.position)
        )

        distance_to_sign = np.linalg.norm(position - closest_sign.position)

        # Calculate required stopping distance
        stopping_distance = self._calculate_stopping_distance(velocity) + self.stopping_distance_buffer

        # Determine if stop/yield is required based on type
        if closest_sign.sign_type == TrafficSignType.STOP_SIGN:
            if distance_to_sign < 30.0 and velocity > 0.5:
                violations.append("APPROACHING_STOP_SIGN_TOO_FAST")
            elif distance_to_sign < 5.0 and velocity > 0.1:
                violations.append("STOP_SIGN_NOT_OBSERVED")
        elif closest_sign.sign_type == TrafficSignType.YIELD_SIGN:
            if distance_to_sign < 25.0 and velocity > 1.0:
                violations.append("APPROACHING_YIELD_SIGN_TOO_FAST")

        return len(violations) == 0, violations, recommendations

    def validate_speed_limits(self, position: np.ndarray, velocity: float,
                            road_speed_limit: float = None) -> Tuple[bool, List[str], Dict[str, Any]]:
        """Validate speed limit compliance"""
        violations = []
        recommendations = {}

        # Find speed limit signs
        speed_signs = [
            sign for sign in self.traffic_signs
            if sign.sign_type == TrafficSignType.SPEED_LIMIT
        ]

        # Determine current speed limit (use road limit if available, otherwise detected sign)
        current_limit = road_speed_limit
        if speed_signs:
            # Use closest speed limit sign if it's within range
            closest_sign = min(
                speed_signs,
                key=lambda x: np.linalg.norm(position - x.position)
            )
            if np.linalg.norm(position - closest_sign.position) < 100.0:  # 100m range
                current_limit = closest_sign.sign_value  # Use the value from the sign

        if current_limit and current_limit > 0:
            speed_excess = velocity - current_limit
            if speed_excess > 5.0:  # Exceeding limit by more than 5 m/s (about 18 km/h)
                violations.append(f"SPEEDING: {velocity:.2f}m/s vs {current_limit:.2f}m/s limit")
                recommendations['recommended_action'] = f'DECREASE_SPEED_TO_{current_limit:.2f}_M_S'
            elif speed_excess > 2.0:  # Exceeding limit by more than 2 m/s
                violations.append(f"MODERATE_SPEEDING: {velocity:.2f}m/s vs {current_limit:.2f}m/s limit")
                recommendations['recommended_action'] = f'ADJUST_SPEED_TO_{current_limit:.2f}_M_S'

        return len(violations) == 0, violations, recommendations

    def validate_pedestrian_crossings(self, position: np.ndarray, velocity: float) -> Tuple[bool, List[str], Dict[str, Any]]:
        """Validate pedestrian crossing safety"""
        violations = []
        recommendations = {}

        # Find pedestrian crossings
        crossings = [
            sign for sign in self.traffic_signs
            if sign.sign_type == TrafficSignType.PEDESTRIAN_CROSSING
        ]

        if not crossings:
            return True, [], {}

        # Get closest crossing
        closest_crossing = min(
            crossings,
            key=lambda x: np.linalg.norm(position - x.position)
        )

        distance_to_crossing = np.linalg.norm(position - closest_crossing.position)

        if distance_to_crossing < 50.0 and velocity > 8.0:  # 8 m/s = ~29 km/h
            violations.append("APPROACHING_PEDESTRIAN_CROSSING_TOO_FAST")
            recommendations['recommended_action'] = 'SLOW_DOWN_FOR_PEDESTRIAN_CROSSING'

        return len(violations) == 0, violations, recommendations

    def _calculate_stopping_distance(self, velocity: float) -> float:
        """Calculate stopping distance based on physics and road conditions"""
        # Basic physics: v² = u² + 2as
        # Rearranging: s = v² / (2a) where a is deceleration
        # Using typical deceleration of 7 m/s² (aggressive but safe)
        typical_deceleration = 7.0  # m/s²

        # Account for road conditions (wet, icy, etc.)
        road_condition_factor = 1.0  # Adjust based on real sensor data if available

        stopping_distance = (velocity ** 2) / (2 * typical_deceleration * road_condition_factor)
        return stopping_distance

    def _is_safe_approach_red_light(self, velocity: float, distance: float, stopping_distance: float) -> bool:
        """Determine if approach to red light is safe"""
        # If we're too close to the light to stop safely, consider it safe (avoiding "stomach braking")
        # If we're far enough to stop safely, then we should slow down
        if distance < stopping_distance:
            # We might not be able to stop in time - check if we're going too fast
            if velocity > 5.0:  # Too fast to stop safely at this distance
                return False
            elif distance < stopping_distance * 0.7:  # Very close, avoid hard braking
                # Allow proceeding if we're too close to stop safely
                return True
        else:
            # We have distance to stop, so we should be slowing down
            return velocity < 10.0  # If going under 10 m/s (36 km/h), it's safer to slow down

        return True

    def get_traffic_context(self, position: np.ndarray) -> Dict[str, Any]:
        """Get comprehensive traffic context around the current position"""
        context = {
            'traffic_lights': [],
            'stop_signs': [],
            'speed_limits': [],
            'other_signs': []
        }

        max_range = 100.0  # Only consider signs within 100m

        for sign in self.traffic_signs:
            distance = np.linalg.norm(position - sign.position)
            if distance > max_range:
                continue

            sign_context = {
                'type': sign.sign_type.value,
                'distance': distance,
                'confidence': sign.confidence,
                'position': sign.position.tolist() if isinstance(sign.position, np.ndarray) else list(sign.position)
            }

            if sign.sign_type in [TrafficSignType.TRAFFIC_LIGHT_RED,
                                TrafficSignType.TRAFFIC_LIGHT_YELLOW,
                                TrafficSignType.TRAFFIC_LIGHT_GREEN]:
                context['traffic_lights'].append(sign_context)
            elif sign.sign_type in [TrafficSignType.STOP_SIGN, TrafficSignType.YIELD_SIGN]:
                context['stop_signs'].append(sign_context)
            elif sign.sign_type == TrafficSignType.SPEED_LIMIT:
                sign_context['value'] = sign.sign_value
                context['speed_limits'].append(sign_context)
            else:
                context['other_signs'].append(sign_context)

        return context

    def update_validation_thresholds(self, current_violations: List[str]):
        """Adaptively update validation thresholds based on driving patterns"""
        if not current_violations:
            # If no violations, we can be slightly more lenient (but maintain safety)
            self.stopping_distance_buffer = min(self.stopping_distance_buffer * 0.95, 5.0)
        else:
            # If violations detected, be more conservative
            self.stopping_distance_buffer = min(self.stopping_distance_buffer * 1.05, 10.0)