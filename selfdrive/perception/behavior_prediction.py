"""
Behavior Prediction for Sunnypilot
Simple behavior prediction for nearby objects
"""
import numpy as np
from typing import Dict, List, Tuple
import math


class BehaviorPredictor:
    """Simple behavior predictor for nearby objects"""

    def __init__(self, prediction_horizon: float = 5.0, dt: float = 0.1):
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.time_steps = int(prediction_horizon / dt)

    def predict_constant_velocity(self, position: np.ndarray, velocity: np.ndarray) -> List[np.ndarray]:
        """Predict trajectory using constant velocity model"""
        trajectory = []
        current_pos = position.copy()

        for _ in range(self.time_steps):
            current_pos = current_pos + velocity * self.dt
            trajectory.append(current_pos.copy())

        return trajectory

    def predict_with_deceleration(self, position: np.ndarray, velocity: np.ndarray, decel: float) -> List[np.ndarray]:
        """Predict trajectory with constant deceleration"""
        trajectory = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        for _ in range(self.time_steps):
            current_pos = current_pos + current_vel * self.dt
            current_vel = current_vel - np.array([decel, 0, 0]) * self.dt
            # Don't allow negative forward velocity
            if current_vel[0] < 0:
                current_vel[0] = 0
            trajectory.append(current_pos.copy())

        return trajectory

    def predict_leads(self, leads_data: List[Dict]) -> Dict[int, List[np.ndarray]]:
        """Predict trajectories for lead vehicles"""
        predictions = {}

        for i, lead in enumerate(leads_data):
            if hasattr(lead, 'dRel') and hasattr(lead, 'vRel'):
                # Convert relative distance and velocity to absolute for prediction
                # This is a simplified approach - real implementation would need more accurate transforms
                pos = np.array([lead.dRel, 0, 0])  # Only longitudinal for now
                vel = np.array([lead.vRel, 0, 0])

                # Use constant velocity model for lead prediction
                trajectory = self.predict_constant_velocity(pos, vel)
                predictions[i] = trajectory

        return predictions

    def calculate_time_to_impact(self, ego_pos: np.ndarray, ego_vel: np.ndarray,
                                obj_pos: np.ndarray, obj_vel: np.ndarray) -> float:
        """Calculate time to potential impact with an object"""
        rel_pos = obj_pos - ego_pos
        rel_vel = obj_vel - ego_vel

        # If objects are moving apart or relative velocity is small, return large value
        if np.dot(rel_pos, rel_vel) >= 0 or np.linalg.norm(rel_vel) < 0.1:
            return float('inf')

        # Project relative position onto relative velocity vector
        time_to_impact = np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
        return max(0, time_to_impact)