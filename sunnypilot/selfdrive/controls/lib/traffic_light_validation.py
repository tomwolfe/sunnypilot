"""
Simple Traffic Sign Validation for Sunnypilot
Minimal implementation for traffic sign detection and validation
"""

import numpy as np
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass
import time
from enum import Enum


class TrafficSignType(Enum):
    """Types of traffic signs"""
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
    timestamp: float = 0.0
    additional_data: Dict[str, Any] = None


class SimpleTrafficValidator:
    """Simple traffic validation system"""

    def __init__(self):
        self.traffic_signs = []
        self.max_signs = 10  # Limit memory usage
        self.validity_threshold = 3.0  # seconds before signs expire

    def add_traffic_sign_data(self, sign_data: TrafficSignData):
        """Add detected traffic sign data"""
        current_time = time.time()
        # Filter out expired signs
        self.traffic_signs = [
            sign for sign in self.traffic_signs
            if current_time - sign.timestamp < self.validity_threshold
        ]

        # Limit number of signs
        if len(self.traffic_signs) >= self.max_signs:
            self.traffic_signs = self.traffic_signs[-(self.max_signs - 1):]

        sign_data.timestamp = current_time
        self.traffic_signs.append(sign_data)

    def validate_traffic_lights(self, position: np.ndarray, velocity: float) -> Tuple[bool, List[str]]:
        """Validate traffic light compliance"""
        violations = []

        # Find traffic lights in range
        traffic_lights = [
            sign for sign in self.traffic_signs
            if sign.sign_type in [TrafficSignType.TRAFFIC_LIGHT_RED,
                                TrafficSignType.TRAFFIC_LIGHT_YELLOW,
                                TrafficSignType.TRAFFIC_LIGHT_GREEN]
        ]

        if not traffic_lights:
            return True, []

        # Get closest traffic light
        closest_light = min(
            traffic_lights,
            key=lambda x: np.linalg.norm(position - x.position)
        )

        distance_to_light = np.linalg.norm(position - closest_light.position)

        # Check red light violation
        if closest_light.sign_type == TrafficSignType.TRAFFIC_LIGHT_RED:
            if velocity > 0.5 and distance_to_light < 50.0:  # 50m threshold
                violations.append("APPROACHING_RED_LIGHT_TOO_FAST")

        return len(violations) == 0, violations

    def validate_stop_signs(self, position: np.ndarray, velocity: float) -> Tuple[bool, List[str]]:
        """Validate stop sign compliance"""
        violations = []

        # Find stop signs
        stop_signs = [
            sign for sign in self.traffic_signs
            if sign.sign_type == TrafficSignType.STOP_SIGN
        ]

        if not stop_signs:
            return True, []

        # Get closest stop sign
        closest_sign = min(
            stop_signs,
            key=lambda x: np.linalg.norm(position - x.position)
        )

        distance_to_sign = np.linalg.norm(position - closest_sign.position)

        if distance_to_sign < 30.0 and velocity > 2.0:  # 30m threshold
            violations.append("APPROACHING_STOP_SIGN_TOO_FAST")

        return len(violations) == 0, violations