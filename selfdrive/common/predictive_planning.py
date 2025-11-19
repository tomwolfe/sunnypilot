"""
Predictive Planning System for Sunnypilot
Simple predictive planning for autonomous driving
"""

import numpy as np
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass
from enum import Enum


class ManeuverType(Enum):
    """Types of maneuvers the planner can execute"""
    LANE_FOLLOW = "lane_follow"
    LANE_CHANGE_LEFT = "lane_change_left"
    LANE_CHANGE_RIGHT = "lane_change_right"


@dataclass
class PredictionState:
    """State prediction for a tracked object"""
    id: int
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    confidence: float  # 0.0 to 1.0


@dataclass
class PlannedTrajectory:
    """Planned trajectory for the ego vehicle"""
    positions: np.ndarray  # shape: [time_steps, 3] - [x, y, z]
    velocities: np.ndarray  # shape: [time_steps, 3] - [vx, vy, vz]
    valid: bool


class SimplePredictivePlanner:
    """Simple predictive planning system"""

    def predict_constant_velocity(self, position: np.ndarray, velocity: np.ndarray, horizon: float = 5.0) -> np.ndarray:
        """Predict trajectory assuming constant velocity motion"""
        time_steps = 10  # Fixed number of steps
        dt = horizon / time_steps
        trajectory = np.zeros((time_steps, 3))

        for i in range(time_steps):
            t = i * dt
            trajectory[i, :2] = position[:2] + velocity[:2] * t  # Only x,y for prediction
            trajectory[i, 2] = position[2]  # Keep z constant

        return trajectory

    def plan_lane_follow(self, ego_state: Dict[str, Any], tracked_objects: List[Dict[str, Any]]) -> PlannedTrajectory:
        """Simple lane following trajectory planner"""
        pos = ego_state['position']
        vel = ego_state['velocity']

        # Simple prediction: continue in current direction
        positions = np.zeros((10, 3))  # 10 time steps
        velocities = np.zeros((10, 3))

        for i in range(10):
            dt = i * 0.5  # 0.5 second intervals
            positions[i] = pos + vel * dt
            velocities[i] = vel

        return PlannedTrajectory(
            positions=positions,
            velocities=velocities,
            valid=True
        )

    def plan_with_obstacles(self, ego_state: Dict[str, Any], tracked_objects: List[Dict[str, Any]]) -> PlannedTrajectory:
        """Plan trajectory considering obstacles"""
        # Check if there are any close obstacles
        ego_pos = ego_state['position']
        ego_vel = ego_state['velocity']

        for obj in tracked_objects:
            obj_pos = np.array([obj.get('dRel', 0.0), obj.get('yRel', 0.0), 0.0])
            distance = np.linalg.norm(ego_pos - obj_pos)

            # If obstacle is close and in our lane, adjust trajectory
            if distance < 50 and abs(obj_pos[1] - ego_pos[1]) < 2.0:
                # For now, keep original plan - a real system would adjust
                pass

        return self.plan_lane_follow(ego_state, tracked_objects)


def get_simple_planner():
    """Get instance of simple planner"""
    return SimplePredictivePlanner()


# Global instance for use across the system
simple_planner = get_simple_planner()


__all__ = [
    "ManeuverType", "PredictionState", "PlannedTrajectory",
    "SimplePredictivePlanner", "get_simple_planner", "simple_planner"
]