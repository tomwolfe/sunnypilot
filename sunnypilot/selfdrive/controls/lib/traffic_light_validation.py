"""
Traffic Light and Sign Validation for Sunnypilot
Provides validation for traffic light and sign detection to improve safety
"""

import numpy as np
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass, field
import time
from enum import Enum


class TrafficSignType(Enum):
    """Types of traffic signs and signals"""
    TRAFFIC_LIGHT_RED = "traffic_light_red"
    TRAFFIC_LIGHT_YELLOW = "traffic_light_yellow"
    TRAFFIC_LIGHT_GREEN = "traffic_light_green"
    STOP_SIGN = "stop_sign"
    SPEED_LIMIT = "speed_limit"
    YIELD_SIGN = "yield_sign"
    PEDESTRIAN_CROSSING = "pedestrian_crossing"
    SCHOOL_ZONE = "school_zone"
    CONSTRUCTION_ZONE = "construction_zone"


@dataclass
class TrafficSignData:
    """Data structure for detected traffic sign"""
    sign_type: TrafficSignType
    distance: float  # Distance to sign in meters
    confidence: float  # Detection confidence (0-1)
    position: np.ndarray  # Position [x, y, z]
    timestamp: float = field(default_factory=time.time) # Add timestamp for expiration checks
    additional_data: Dict[str, Any] = field(default_factory=dict) # For storing extra info like speed limit value


class TrafficValidator:
    """Traffic validation system for traffic lights and signs"""

    def __init__(self, max_signs: int = 50, validity_threshold: float = 5.0, red_light_distance_threshold: float = 50.0, stop_sign_distance_threshold: float = 30.0):
        self.traffic_signs: List[TrafficSignData] = []
        self.max_signs = max_signs  # Limit to prevent memory issues
        self.validity_threshold = validity_threshold  # seconds before signs expire
        self.red_light_distance_threshold = red_light_distance_threshold  # meters for red light checking
        self.stop_sign_distance_threshold = stop_sign_distance_threshold  # meters for stop sign checking

    def add_traffic_sign_data(self, sign_data: TrafficSignData):
        """Add detected traffic sign data to validation"""
        current_time = time.time()
        # Filter out expired signs
        self.traffic_signs = [
            sign for sign in self.traffic_signs
            if current_time - sign.timestamp < self.validity_threshold
        ]

        # Limit number of signs to prevent memory issues
        if len(self.traffic_signs) >= self.max_signs:
            self.traffic_signs = self.traffic_signs[-(self.max_signs - 1):]

        self.traffic_signs.append(sign_data)

    def validate_traffic_lights(self, position: np.ndarray, velocity: float) -> Tuple[bool, List[str], str]:
        """Validate traffic light compliance"""
        violations = []
        recommended_action = "NONE"

        # Find traffic lights in range
        traffic_lights = [
            sign for sign in self.traffic_signs
            if sign.sign_type in [TrafficSignType.TRAFFIC_LIGHT_RED,
                                TrafficSignType.TRAFFIC_LIGHT_YELLOW,
                                TrafficSignType.TRAFFIC_LIGHT_GREEN]
        ]

        if not traffic_lights:
            return True, [], recommended_action

        # Get closest traffic light
        closest_light = min(
            traffic_lights,
            key=lambda x: np.linalg.norm(position - x.position)
        )

        distance_to_light = np.linalg.norm(position - closest_light.position)

        # Check if approach is appropriate for light status
        if closest_light.sign_type == TrafficSignType.TRAFFIC_LIGHT_RED:
            if velocity > 0.5 and distance_to_light < self.red_light_distance_threshold:
                violations.append("APPROACHING_RED_LIGHT_TOO_FAST")
                recommended_action = "BRAKE_IMMEDIATELY"

        return len(violations) == 0, violations, recommended_action

    def validate_stop_signs(self, position: np.ndarray, velocity: float) -> Tuple[bool, List[str], str]:
        """Validate stop sign compliance"""
        violations = []
        recommended_action = "NONE"

        # Find stop signs in range
        stop_signs = [
            sign for sign in self.traffic_signs
            if sign.sign_type == TrafficSignType.STOP_SIGN
        ]

        if not stop_signs:
            return True, [], recommended_action

        # Get closest stop sign
        closest_sign = min(
            stop_signs,
            key=lambda x: np.linalg.norm(position - x.position)
        )

        distance_to_sign = np.linalg.norm(position - closest_sign.position)

        # Check if we're approaching stop sign appropriately
        if distance_to_sign < self.stop_sign_distance_threshold and velocity > 2.0:
            violations.append("APPROACHING_STOP_SIGN_TOO_FAST")
            recommended_action = "BRAKE_FOR_STOP"

        return len(violations) == 0, violations, recommended_action


def create_traffic_validator(max_signs: int = 50, validity_threshold: float = 5.0,
                           red_light_distance_threshold: float = 50.0,
                           stop_sign_distance_threshold: float = 30.0) -> TrafficValidator:
    """Factory function to create traffic validator"""
    return TrafficValidator(
        max_signs=max_signs,
        validity_threshold=validity_threshold,
        red_light_distance_threshold=red_light_distance_threshold,
        stop_sign_distance_threshold=stop_sign_distance_threshold
    )