"""
Enhanced Behavior Prediction for Sunnypilot
Advanced behavior prediction for nearby objects with realistic models
"""
import numpy as np
from typing import Dict, List, Tuple, Optional
import math


class BehaviorPredictor:
    """Enhanced behavior predictor for nearby objects with multiple prediction models"""

    def __init__(self, prediction_horizon: float = 8.0, dt: float = 0.05):
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.time_steps = int(prediction_horizon / dt)
        self.max_acceleration = 3.0  # m/s^2, typical max for vehicles
        self.max_deceleration = -5.0  # m/s^2, typical emergency braking

    def predict_constant_velocity(self, position: np.ndarray, velocity: np.ndarray) -> List[np.ndarray]:
        """Predict trajectory using constant velocity model"""
        trajectory = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        for _ in range(self.time_steps):
            current_pos = current_pos + current_vel * self.dt
            trajectory.append(current_pos.copy())

        return trajectory

    def predict_with_acceleration(self, position: np.ndarray, velocity: np.ndarray, acceleration: float) -> List[np.ndarray]:
        """Predict trajectory with constant acceleration"""
        trajectory = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        for _ in range(self.time_steps):
            current_pos = current_pos + current_vel * self.dt + 0.5 * acceleration * self.dt**2
            current_vel = current_vel + acceleration * self.dt
            trajectory.append(current_pos.copy())

        return trajectory

    def predict_with_deceleration(self, position: np.ndarray, velocity: np.ndarray, decel: float) -> List[np.ndarray]:
        """Predict trajectory with constant deceleration (enhanced safety)"""
        trajectory = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        for _ in range(self.time_steps):
            current_pos = current_pos + current_vel * self.dt
            current_vel = current_vel - np.array([abs(decel), 0, 0]) * self.dt
            # Don't allow negative forward velocity
            if current_vel[0] < 0:
                current_vel[0] = 0
                # Stop moving forward if velocity is zero
                current_pos[0] = trajectory[-1][0] if trajectory else current_pos[0]
            trajectory.append(current_pos.copy())

        return trajectory

    def predict_with_lead_behavior(self, ego_state: Dict, lead_state: Dict) -> List[np.ndarray]:
        """
        Predict lead vehicle behavior based on relative position and velocity
        More sophisticated than simple constant velocity
        """
        if not (hasattr(lead_state, 'dRel') and hasattr(lead_state, 'vRel')):
            return self.predict_constant_velocity(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

        # Start with relative position and velocity
        pos = np.array([lead_state.dRel, 0.0, 0.0])  # Only longitudinal for now
        vel = np.array([lead_state.vRel, 0.0, 0.0])

        # Use a more sophisticated model based on current conditions
        trajectory = []
        current_pos = pos.copy()
        current_vel = vel.copy()

        # Determine likely behavior based on spacing and relative velocity
        safe_distance = max(30.0, ego_state.get('vEgo', 0.0) * 2.0)  # Adjust based on ego speed
        relative_distance = lead_state.dRel

        for i in range(self.time_steps):
            # Calculate adaptive acceleration based on distance
            acceleration = 0.0
            if relative_distance < safe_distance * 0.5:  # Too close, decelerate
                acceleration = min(0.0, self.max_deceleration * 0.7)  # Heavy deceleration
            elif relative_distance < safe_distance * 0.8:  # Getting close, decelerate gently
                acceleration = min(0.0, self.max_deceleration * 0.3)  # Gentle deceleration
            elif relative_distance > safe_distance * 1.5:  # Too far, accelerate gently
                acceleration = min(self.max_acceleration * 0.3, 0.5)  # Gentle acceleration
            # Otherwise maintain current velocity

            # Update position and velocity
            current_pos = current_pos + current_vel * self.dt + 0.5 * np.array([acceleration, 0, 0]) * self.dt**2
            current_vel = current_vel + np.array([acceleration, 0, 0]) * self.dt

            # Limit velocity to realistic bounds
            current_vel[0] = max(0.0, current_vel[0])  # No negative velocity

            trajectory.append(current_pos.copy())

        return trajectory

    def predict_leads(self, ego_state: Dict, leads_data: List[Dict]) -> Dict[int, List[np.ndarray]]:
        """Predict trajectories for lead vehicles with enhanced models"""
        predictions = {}

        for i, lead in enumerate(leads_data):
            if hasattr(lead, 'dRel') and hasattr(lead, 'vRel'):
                # Use more sophisticated prediction model
                trajectory = self.predict_with_lead_behavior(ego_state, lead)
                predictions[i] = trajectory

        return predictions

    def calculate_time_to_impact(self, ego_pos: np.ndarray, ego_vel: np.ndarray,
                                obj_pos: np.ndarray, obj_vel: np.ndarray) -> float:
        """Calculate time to potential impact with an object (enhanced for safety)"""
        rel_pos = obj_pos - ego_pos
        rel_vel = obj_vel - ego_vel

        # If objects are moving apart or relative velocity is small, return large value
        rel_speed = np.linalg.norm(rel_vel)
        if np.dot(rel_pos, rel_vel) >= 0 or rel_speed < 0.1:
            return float('inf')

        # Project relative position onto relative velocity vector
        time_to_impact = np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
        return max(0, time_to_impact)

    def estimate_intent(self, trajectory: List[np.ndarray]) -> str:
        """Estimate the intent of the object based on predicted trajectory"""
        if len(trajectory) < 2:
            return "unknown"

        # Calculate average velocity over the prediction window
        total_displacement = trajectory[-1] - trajectory[0]
        avg_velocity = total_displacement / self.prediction_horizon

        # Determine intent based on velocity
        speed = np.linalg.norm(avg_velocity)
        if speed < 0.5:
            return "stopping"
        elif avg_velocity[0] > 1.0:
            return "accelerating"
        elif avg_velocity[0] < -0.5:
            return "decelerating"
        else:
            return "maintaining_speed"

    def predict_multiple_scenarios(self, position: np.ndarray, velocity: np.ndarray) -> Dict[str, List[np.ndarray]]:
        """Predict multiple behavior scenarios for the same object"""
        scenarios = {}

        # Constant velocity (most likely scenario)
        scenarios['constant_velocity'] = self.predict_constant_velocity(position, velocity)

        # Deceleration scenario
        scenarios['decelerating'] = self.predict_with_deceleration(position, velocity, self.max_deceleration * 0.6)

        # Acceleration scenario
        scenarios['accelerating'] = self.predict_with_acceleration(position, velocity, self.max_acceleration * 0.5)

        return scenarios