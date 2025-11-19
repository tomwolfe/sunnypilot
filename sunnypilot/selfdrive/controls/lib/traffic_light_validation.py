"""
Traffic Light and Sign Validation for Sunnypilot
Provides validation for traffic light and sign detection to improve safety
"""

import numpy as np
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass
import time
from enum import Enum


class TrafficSignType(Enum):
    """Types of traffic signs and signals"""
    TRAFFIC_LIGHT_RED = "traffic_light_red"
    TRAFFIC_LIGHT_YELLOW = "traffic_light_yellow"
    TRAFFIC_LIGHT_GREEN = "traffic_light_green"
    STOP_SIGN = "stop_sign"
    SPEED_LIMIT = "speed_limit"


@dataclass
class TrafficSignData:
    """Data structure for detected traffic sign"""
    sign_type: TrafficSignType
    distance: float  # Distance to sign in meters
    confidence: float  # Detection confidence (0-1)
    position: np.ndarray  # Position [x, y, z]


class TrafficValidator:
    """Traffic validation system for traffic lights and signs"""

    def __init__(self):
        self.traffic_signs: List[TrafficSignData] = []

    def add_traffic_sign_data(self, sign_data: TrafficSignData):
        """Add detected traffic sign data to validation"""
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
            if velocity > 0.5 and distance_to_light < 50.0:  # 50m buffer for red light
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
        if distance_to_sign < 30.0 and velocity > 2.0:
            violations.append("APPROACHING_STOP_SIGN_TOO_FAST")
            recommended_action = "BRAKE_FOR_STOP"

        return len(violations) == 0, violations, recommended_action


def create_traffic_validator() -> TrafficValidator:
    """Factory function to create traffic validator"""
    return TrafficValidator()