"""
Predictive Planning System for Sunnypilot
Simple predictive planning for autonomous driving
"""

import numpy as np
from typing import Dict, List, Tuple, Any, Optional
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
    maneuver_type: ManeuverType = ManeuverType.LANE_FOLLOW  # Added maneuver type


class SimplePredictivePlanner:
    """Simple predictive planning system"""

    def __init__(self):
        self.horizon = 5.0  # Planning horizon in seconds
        self.time_steps = 10  # Number of time steps in planning

    def predict_constant_velocity(self, position: np.ndarray, velocity: np.ndarray, horizon: float = 5.0) -> np.ndarray:
        """Predict trajectory assuming constant velocity motion"""
        try:
            time_steps = 10  # Fixed number of steps
            dt = horizon / time_steps
            trajectory = np.zeros((time_steps, 3))

            for i in range(time_steps):
                t = i * dt
                trajectory[i, :2] = position[:2] + velocity[:2] * t  # Only x,y for prediction
                trajectory[i, 2] = position[2] if len(position) > 2 else 0.0  # Keep z constant or default to 0

            return trajectory
        except Exception as e:
            print(f"Error in constant velocity prediction: {e}")
            # Return default trajectory in case of error
            return np.zeros((self.time_steps, 3))

    def plan_lane_follow(self, ego_state: Dict[str, Any], tracked_objects: List[Dict[str, Any]]) -> PlannedTrajectory:
        """Simple lane following trajectory planner"""
        try:
            # Extract ego state with defaults
            pos = ego_state.get('position', np.array([0.0, 0.0, 0.0]))
            vel = ego_state.get('velocity', np.array([0.0, 0.0, 0.0]))

            # Ensure position and velocity are proper numpy arrays
            pos = np.asarray(pos)
            vel = np.asarray(vel)

            if pos.shape != (3,):
                pos = np.pad(pos, (0, 3 - len(pos)), mode='constant') if len(pos) < 3 else pos[:3]

            if vel.shape != (3,):
                vel = np.pad(vel, (0, 3 - len(vel)), mode='constant') if len(vel) < 3 else vel[:3]

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
                valid=True,
                maneuver_type=ManeuverType.LANE_FOLLOW
            )
        except Exception as e:
            print(f"Error in lane follow planning: {e}")
            # Return default trajectory in case of error
            return PlannedTrajectory(
                positions=np.zeros((10, 3)),
                velocities=np.zeros((10, 3)),
                valid=False,
                maneuver_type=ManeuverType.LANE_FOLLOW
            )

    def plan_with_obstacles(self, ego_state: Dict[str, Any], tracked_objects: List[Dict[str, Any]]) -> PlannedTrajectory:
        """Plan trajectory considering obstacles"""
        try:
            # Check if there are any close obstacles
            pos = ego_state.get('position', np.array([0.0, 0.0, 0.0]))
            vel = ego_state.get('velocity', np.array([0.0, 0.0, 0.0]))

            # Ensure position and velocity are proper numpy arrays
            ego_pos = np.asarray(pos)
            ego_vel = np.asarray(vel)

            if ego_pos.shape != (3,):
                ego_pos = np.pad(ego_pos, (0, 3 - len(ego_pos)), mode='constant') if len(ego_pos) < 3 else ego_pos[:3]

            if ego_vel.shape != (3,):
                ego_vel = np.pad(ego_vel, (0, 3 - len(ego_vel)), mode='constant') if len(ego_vel) < 3 else ego_vel[:3]

            # Track closest obstacle to determine if lane change is needed
            closest_obstacle_dist = float('inf')
            for obj in tracked_objects:
                obj_pos = np.array([
                    obj.get('dRel', 0.0),
                    obj.get('yRel', 0.0),
                    0.0
                ])

                distance = np.linalg.norm(ego_pos[:2] - obj_pos[:2])  # Only consider x,y plane

                # If obstacle is close and in our lane, it might affect our plan
                if distance < 50 and abs(obj_pos[1] - ego_pos[1]) < 2.0:
                    closest_obstacle_dist = min(closest_obstacle_dist, distance)

                    # If very close, consider deceleration
                    if distance < 25 and obj.get('vRel', 0.0) < -2.0:  # Obstacle approaching quickly
                        # Reduce speed slightly to maintain safe distance
                        vel_factor = max(0.7, distance / 25.0)  # Slow down as we get closer
                        vel = ego_vel * vel_factor

            # Return the appropriate plan based on obstacle detection
            if closest_obstacle_dist < 25:
                # Adjust plan for obstacle
                positions = np.zeros((10, 3))
                velocities = np.zeros((10, 3))

                for i in range(10):
                    dt = i * 0.5  # 0.5 second intervals
                    positions[i] = ego_pos + vel * dt
                    velocities[i] = vel

                return PlannedTrajectory(
                    positions=positions,
                    velocities=velocities,
                    valid=True,
                    maneuver_type=ManeuverType.LANE_FOLLOW  # For now, just slow down, don't change lanes
                )
            else:
                # No obstacles, continue as normal
                return self.plan_lane_follow(ego_state, tracked_objects)
        except Exception as e:
            print(f"Error in obstacle-aware planning: {e}")
            # Return basic lane follow plan in case of error
            return self.plan_lane_follow(ego_state, tracked_objects)

    def plan_lane_change(self, ego_state: Dict[str, Any], tracked_objects: List[Dict[str, Any]],
                        direction: ManeuverType = ManeuverType.LANE_CHANGE_LEFT) -> PlannedTrajectory:
        """Plan a lane change maneuver"""
        try:
            pos = ego_state.get('position', np.array([0.0, 0.0, 0.0]))
            vel = ego_state.get('velocity', np.array([0.0, 0.0, 0.0]))

            # Ensure position and velocity are proper numpy arrays
            pos = np.asarray(pos)
            vel = np.asarray(vel)

            if pos.shape != (3,):
                pos = np.pad(pos, (0, 3 - len(pos)), mode='constant') if len(pos) < 3 else pos[:3]

            if vel.shape != (3,):
                vel = np.pad(vel, (0, 3 - len(vel)), mode='constant') if len(vel) < 3 else vel[:3]

            # Determine lane change direction
            lane_change_offset = 3.7  # Standard lane width in meters
            if direction == ManeuverType.LANE_CHANGE_RIGHT:
                lane_change_offset = -lane_change_offset

            # Create trajectory for lane change with smooth transition
            positions = np.zeros((10, 3))
            velocities = np.zeros((10, 3))

            for i in range(10):
                dt = i * 0.5  # 0.5 second intervals
                # Gradually change lateral position
                lateral_ratio = min(1.0, (dt / 4.0))  # Smooth transition over 4 seconds
                positions[i, 0] = pos[0] + vel[0] * dt  # Maintain longitudinal motion
                positions[i, 1] = pos[1] + lane_change_offset * (1 - np.exp(-lateral_ratio * 3))  # Smooth lateral transition
                positions[i, 2] = pos[2]  # Maintain vertical position

                velocities[i] = vel  # Maintain velocity for now

            return PlannedTrajectory(
                positions=positions,
                velocities=velocities,
                valid=True,
                maneuver_type=direction
            )
        except Exception as e:
            print(f"Error in lane change planning: {e}")
            # Return default trajectory in case of error
            return PlannedTrajectory(
                positions=np.zeros((10, 3)),
                velocities=np.zeros((10, 3)),
                valid=False,
                maneuver_type=ManeuverType.LANE_FOLLOW
            )


def get_simple_planner():
    """Get instance of simple planner"""
    return SimplePredictivePlanner()


def get_predictive_planner():  # Added this function that was referenced in the simulation code
    """Get instance of predictive planner - alias for compatibility"""
    return SimplePredictivePlanner()


# Global instance for use across the system
simple_planner = get_simple_planner()


__all__ = [
    "ManeuverType", "PredictionState", "PlannedTrajectory",
    "SimplePredictivePlanner", "get_simple_planner", "get_predictive_planner", "simple_planner"
]