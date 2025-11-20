"""
Enhanced Behavior Prediction for Sunnypilot
Advanced behavior prediction with realistic physics and interaction modeling
"""
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import math


class ObjectBehaviorType(Enum):
    """Types of object behaviors for prediction"""
    STATIC = "static"  # Not moving
    CONSTANT_VELOCITY = "constant_velocity"
    CONSTANT_ACCELERATION = "constant_acceleration"
    LANE_FOLLOWING = "lane_following"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    TRAFFIC_LIGHT_RESPONSE = "traffic_light_response"
    VEHICLE_FOLLOWING = "vehicle_following"


@dataclass
class PredictedObject:
    """Enhanced predicted object with behavior type and interaction modeling"""
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    acceleration: np.ndarray  # [ax, ay, az]
    predicted_trajectory: List[np.ndarray]  # List of future positions
    confidence: float
    behavior_type: ObjectBehaviorType = ObjectBehaviorType.CONSTANT_VELOCITY
    interaction_factor: float = 1.0  # Factor accounting for interaction with other objects
    predicted_heading: List[float] = None  # Predicted headings over time
    predicted_velocity_profile: List[np.ndarray] = None  # Velocity at each time step


class EnhancedBehaviorPredictor:
    """Advanced behavior predictor with multiple prediction models and interaction awareness"""

    def __init__(self, prediction_horizon: float = 5.0, dt: float = 0.05):
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.time_steps = int(prediction_horizon / dt)
        self.max_deceleration = 8.0  # Maximum deceleration (m/s^2)
        self.min_safe_distance = 20.0  # Minimum safe distance (m)
        self.response_time = 1.0  # Typical reaction time (s)

    def predict_object_trajectory(self, position: np.ndarray, velocity: np.ndarray,
                                  acceleration: np.ndarray = None,
                                  behavior_type: ObjectBehaviorType = ObjectBehaviorType.CONSTANT_VELOCITY,
                                  environment_context: Dict = None) -> PredictedObject:
        """Predict trajectory for a single object with behavior modeling"""
        if acceleration is None:
            acceleration = np.array([0.0, 0.0, 0.0])

        # Select prediction model based on behavior type
        if behavior_type == ObjectBehaviorType.VEHICLE_FOLLOWING:
            trajectory, velocities = self._predict_vehicle_following(
                position, velocity, acceleration, environment_context
            )
        elif behavior_type == ObjectBehaviorType.LANE_FOLLOWING:
            trajectory, velocities = self._predict_lane_following(
                position, velocity, acceleration
            )
        elif behavior_type == ObjectBehaviorType.TRAFFIC_LIGHT_RESPONSE:
            trajectory, velocities = self._predict_traffic_light_response(
                position, velocity, acceleration, environment_context
            )
        elif behavior_type == ObjectBehaviorType.OBSTACLE_AVOIDANCE:
            trajectory, velocities = self._predict_obstacle_avoidance(
                position, velocity, acceleration, environment_context
            )
        else:
            # Use constant acceleration model as default
            trajectory, velocities = self._predict_constant_acceleration(
                position, velocity, acceleration
            )

        # Calculate confidence based on prediction horizon and behavior type
        confidence = self._calculate_prediction_confidence(behavior_type, len(trajectory))

        # Calculate headings at each time step
        headings = self._calculate_headings(velocities)

        return PredictedObject(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            predicted_trajectory=trajectory,
            confidence=confidence,
            behavior_type=behavior_type,
            interaction_factor=self._calculate_interaction_factor(environment_context),
            predicted_heading=headings,
            predicted_velocity_profile=velocities
        )

    def _predict_constant_acceleration(self, position: np.ndarray, velocity: np.ndarray,
                                     acceleration: np.ndarray) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Predict using constant acceleration model"""
        trajectory = []
        velocities = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        for _ in range(self.time_steps):
            # More accurate physics integration: pos = pos + vel*dt + 0.5*acc*dt^2
            current_pos = current_pos + current_vel * self.dt + 0.5 * acceleration * self.dt**2
            current_vel = current_vel + acceleration * self.dt
            trajectory.append(current_pos.copy())
            velocities.append(current_vel.copy())

        return trajectory, velocities

    def _predict_vehicle_following(self, position: np.ndarray, velocity: np.ndarray,
                                  acceleration: np.ndarray, environment_context: Dict) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Predict vehicle following behavior with safe distance maintenance"""
        trajectory = []
        velocities = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        # Get leading vehicle information if available
        leading_vehicle = environment_context.get('leading_vehicle') if environment_context else None
        safe_distance = self.min_safe_distance

        if leading_vehicle:
            # Use intelligent driver model (IDM) for realistic vehicle following
            for i in range(self.time_steps):
                # Calculate distance to leading vehicle
                if i == 0:
                    dist_to_leading = np.linalg.norm(current_pos - leading_vehicle['position'])
                else:
                    # Estimate leading vehicle position at this time step
                    lead_pos_est = (leading_vehicle['position'] +
                                  leading_vehicle['velocity'] * self.dt * i)
                    dist_to_leading = np.linalg.norm(current_pos - lead_pos_est)

                # Calculate desired acceleration based on IDM
                desired_accel = self._calculate_idm_acceleration(
                    current_vel[0],  # longitudinal velocity
                    dist_to_leading,
                    leading_vehicle.get('velocity', np.array([0, 0, 0]))[0] if leading_vehicle else 0
                )

                # Apply the acceleration
                current_acc = np.array([desired_accel, 0, 0])  # Only longitudinal for now
                current_pos = current_pos + current_vel * self.dt + 0.5 * current_acc * self.dt**2
                current_vel = current_vel + current_acc * self.dt

                trajectory.append(current_pos.copy())
                velocities.append(current_vel.copy())
        else:
            # If no leading vehicle, follow normal trajectory
            return self._predict_constant_acceleration(position, velocity, acceleration)

        return trajectory, velocities

    def _calculate_idm_acceleration(self, v: float, s: float, v_lead: float) -> float:
        """Calculate acceleration using Intelligent Driver Model"""
        # IDM parameters
        v0 = 30.0  # Desired velocity (m/s)
        T = 1.5    # Safe time headway (s)
        a = 1.0    # Maximum acceleration (m/s^2)
        b = 1.5    # Comfortable deceleration (m/s^2)
        s0 = 5.0   # Minimum distance (m)
        delta = 4.0  # Exponent

        # Calculate desired gap
        v_diff = v - v_lead
        s_star = s0 + max(0, v * T + (v * v_diff) / (2 * math.sqrt(a * b)))

        # Calculate acceleration
        acceleration = a * (1 - (v / v0)**delta - (s_star / max(s, 0.1))**2)

        # Limit acceleration to reasonable bounds
        acceleration = max(-self.max_deceleration, min(self.max_deceleration, acceleration))

        return acceleration

    def _predict_lane_following(self, position: np.ndarray, velocity: np.ndarray,
                               acceleration: np.ndarray) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Predict lane following behavior"""
        # For now, just follow the same trajectory but with lane-aware behavior
        trajectory, velocities = self._predict_constant_acceleration(position, velocity, acceleration)

        # Add small lateral adjustments for lane keeping
        for i, (pos, vel) in enumerate(zip(trajectory, velocities)):
            # Apply lane-keeping corrections based on position relative to lane center
            # This would use actual lane detection in a real implementation
            pass

        return trajectory, velocities

    def _predict_traffic_light_response(self, position: np.ndarray, velocity: np.ndarray,
                                       acceleration: np.ndarray, environment_context: Dict) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Predict response to traffic lights"""
        trajectory = []
        velocities = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        # Get traffic light information if available
        traffic_light = environment_context.get('traffic_light') if environment_context else None

        if traffic_light:
            light_distance = np.linalg.norm(current_pos - traffic_light['position'])
            time_to_light = light_distance / max(current_vel[0], 1.0)  # Avoid division by zero

            # Check if we should prepare to stop
            if (traffic_light.get('state') == 'red' and
                time_to_light < 10.0 and  # Less than 10 seconds to light
                current_vel[0] > 5.0):  # Going fast enough to need to slow down

                # Apply deceleration to stop before light
                stopping_distance = light_distance - 5.0  # 5m buffer before light
                required_deceleration = (current_vel[0]**2) / (2 * stopping_distance) if stopping_distance > 0 else 0

                for i in range(self.time_steps):
                    # Gradually decelerate
                    current_acc = np.array([-min(required_deceleration, self.max_deceleration), 0, 0])
                    current_pos = current_pos + current_vel * self.dt + 0.5 * current_acc * self.dt**2
                    current_vel = current_vel + current_acc * self.dt

                    # Don't let velocity go negative
                    if current_vel[0] < 0:
                        current_vel[0] = 0

                    trajectory.append(current_pos.copy())
                    velocities.append(current_vel.copy())
            else:
                # Continue with normal trajectory
                return self._predict_constant_acceleration(position, velocity, acceleration)
        else:
            # Continue with normal trajectory
            return self._predict_constant_acceleration(position, velocity, acceleration)

        return trajectory, velocities

    def _predict_obstacle_avoidance(self, position: np.ndarray, velocity: np.ndarray,
                                   acceleration: np.ndarray, environment_context: Dict) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Predict obstacle avoidance behavior"""
        trajectory = []
        velocities = []
        current_pos = position.copy()
        current_vel = velocity.copy()

        # Get obstacle information if available
        obstacles = environment_context.get('obstacles', []) if environment_context else []

        if obstacles:
            # For simplicity, assume first obstacle is the one to avoid
            obstacle = obstacles[0] if obstacles else None

            if obstacle:
                for i in range(self.time_steps):
                    # Calculate distance to obstacle
                    dist_to_obstacle = np.linalg.norm(current_pos - obstacle['position'])

                    if dist_to_obstacle < 20.0:  # Within 20m, start avoiding
                        # Apply lateral movement to avoid obstacle
                        # This is a simplified model - real implementation would be more complex
                        avoidance_vector = np.array([0, 5.0, 0])  # Move laterally at 5 m/s
                        current_pos = current_pos + current_vel * self.dt + avoidance_vector * self.dt
                    else:
                        current_pos = current_pos + current_vel * self.dt + 0.5 * acceleration * self.dt**2

                    current_vel = current_vel + acceleration * self.dt
                    trajectory.append(current_pos.copy())
                    velocities.append(current_vel.copy())
            else:
                # Continue with normal trajectory
                return self._predict_constant_acceleration(position, velocity, acceleration)
        else:
            # Continue with normal trajectory
            return self._predict_constant_acceleration(position, velocity, acceleration)

        return trajectory, velocities

    def _calculate_prediction_confidence(self, behavior_type: ObjectBehaviorType, prediction_length: int) -> float:
        """Calculate prediction confidence based on behavior type and time horizon"""
        # Start with high confidence
        confidence = 0.95

        # Reduce confidence for more complex behaviors
        if behavior_type in [ObjectBehaviorType.OBSTACLE_AVOIDANCE, ObjectBehaviorType.TRAFFIC_LIGHT_RESPONSE]:
            confidence *= 0.8
        elif behavior_type == ObjectBehaviorType.VEHICLE_FOLLOWING:
            confidence *= 0.85

        # Reduce confidence with longer prediction horizons
        time_horizon_factor = max(0.5, 1.0 - (prediction_length * self.dt) / (2 * self.prediction_horizon))
        confidence *= time_horizon_factor

        return max(0.1, confidence)  # Minimum confidence of 0.1

    def _calculate_headings(self, velocities: List[np.ndarray]) -> List[float]:
        """Calculate heading angles from velocity vectors"""
        headings = []
        for vel in velocities:
            # Calculate heading from velocity vector (in radians)
            heading = math.atan2(vel[1], vel[0])  # y and x components
            headings.append(heading)
        return headings

    def _calculate_interaction_factor(self, environment_context: Dict) -> float:
        """Calculate how much other objects affect this prediction"""
        if not environment_context:
            return 1.0

        # Count nearby objects that might affect behavior
        nearby_objects = environment_context.get('nearby_objects', [])
        return min(1.0, 1.0 / (1 + len(nearby_objects) * 0.1))

    def predict_multiple_objects(self, objects_data: List[Dict], environment_context: Dict = None) -> List[PredictedObject]:
        """Predict trajectories for multiple objects with interaction modeling"""
        predictions = []

        for obj_data in objects_data:
            pos = np.array(obj_data.get('position', [0, 0, 0]))
            vel = np.array(obj_data.get('velocity', [0, 0, 0]))
            acc = np.array(obj_data.get('acceleration', [0, 0, 0]))
            behavior_type = obj_data.get('behavior_type', ObjectBehaviorType.CONSTANT_VELOCITY)

            prediction = self.predict_object_trajectory(pos, vel, acc, behavior_type, environment_context)
            predictions.append(prediction)

        return predictions

    def predict_with_interaction(self, objects_data: List[Dict], environment_context: Dict = None) -> List[PredictedObject]:
        """Predict trajectories considering interactions between objects"""
        predictions = []
        all_positions = []
        all_velocities = []

        # First pass: get all current positions and velocities
        for obj_data in objects_data:
            pos = np.array(obj_data.get('position', [0, 0, 0]))
            vel = np.array(obj_data.get('velocity', [0, 0, 0]))
            all_positions.append(pos)
            all_velocities.append(vel)

        # Calculate interaction context for each object
        for i, obj_data in enumerate(objects_data):
            pos = np.array(obj_data.get('position', [0, 0, 0]))
            vel = np.array(obj_data.get('velocity', [0, 0, 0]))
            acc = np.array(obj_data.get('acceleration', [0, 0, 0]))
            behavior_type = obj_data.get('behavior_type', ObjectBehaviorType.CONSTANT_VELOCITY)

            # Create context including other objects
            obj_context = environment_context if environment_context else {}
            other_objects = []
            for j, (other_pos, other_vel) in enumerate(zip(all_positions, all_velocities)):
                if i != j:  # Don't include self
                    # Calculate distance to other object
                    dist = np.linalg.norm(pos - other_pos)
                    if dist < 50.0:  # Only consider nearby objects
                        other_objects.append({
                            'position': other_pos,
                            'velocity': other_vel,
                            'distance': dist
                        })

            obj_context['nearby_objects'] = other_objects

            prediction = self.predict_object_trajectory(pos, vel, acc, behavior_type, obj_context)
            predictions.append(prediction)

        return predictions