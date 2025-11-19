#!/usr/bin/env python3
"""
Behavior Prediction System for Sunnypilot
Predicts future actions of detected objects to enable proactive planning
"""
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import math
import time
from collections import deque, defaultdict

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.perception.kalman_tracker import TrackedObject


@dataclass
class PredictionResult:
    """Result of behavior prediction"""
    predicted_trajectory: np.ndarray  # [num_steps, 3] - future positions
    confidence: float  # Confidence in the prediction
    behavior_class: str  # Predicted behavior (lane_following, lane_changing, stopping, etc.)
    time_horizon: float  # Time span for prediction in seconds


@dataclass
class VehicleState:
    """Current state of a vehicle for behavior prediction"""
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    acceleration: np.ndarray  # [ax, ay, az]
    heading: float  # Heading angle in radians
    object_type: str  # car, truck, pedestrian, etc.
    lane_position: Optional[int]  # Current lane (0=center, -1=left, 1=right)
    intended_lane: Optional[int]  # Target lane for lane change


class BehaviorPredictor:
    """
    Predicts future behaviors of tracked objects based on their movement patterns
    and contextual information
    """
    
    def __init__(self, prediction_horizon: float = 5.0, time_step: float = 0.5):
        self.prediction_horizon = prediction_horizon  # seconds
        self.time_step = time_step  # seconds
        self.num_steps = int(prediction_horizon / time_step)
        
        # History for behavior pattern recognition
        self.object_history = defaultdict(lambda: deque(maxlen=20))  # Keep last 20 states
        self.lane_positions = {}  # Track lane positions of objects
        
        # Behavior classification thresholds
        self.lane_change_threshold = 0.3  # m/s lateral velocity to consider lane change
        self.stopping_threshold = -1.5  # m/s² deceleration to consider stopping
        self.speeding_threshold = 15.0  # m/s above which behavior changes
        
    def predict_behavior(self, tracked_object: TrackedObject) -> PredictionResult:
        """Predict the future behavior of a tracked object"""
        # Extract current state from the tracked object
        current_pos = tracked_object.state[:3]  # x, y, z
        current_vel = tracked_object.state[3:6]  # vx, vy, vz  
        current_acc = tracked_object.state[6:9]  # ax, ay, az
        
        # Calculate heading from velocity vector
        heading = np.arctan2(current_vel[1], current_vel[0]) if np.linalg.norm(current_vel[:2]) > 0.1 else 0.0
        
        # Create vehicle state
        vehicle_state = VehicleState(
            position=current_pos,
            velocity=current_vel,
            acceleration=current_acc,
            heading=heading,
            object_type=tracked_object.class_name,
            lane_position=self._estimate_lane_position(tracked_object),
            intended_lane=self._estimate_intended_lane(tracked_object)
        )
        
        # Add to history for pattern recognition
        self._update_history(tracked_object.id, vehicle_state)
        
        # Determine behavior class based on historical patterns and current state
        behavior_class = self._classify_behavior(tracked_object.id, vehicle_state)
        
        # Generate trajectory based on behavior class
        predicted_trajectory = self._generate_trajectory(vehicle_state, behavior_class)
        
        # Calculate confidence based on tracking quality and historical consistency
        confidence = self._calculate_prediction_confidence(tracked_object, behavior_class)
        
        return PredictionResult(
            predicted_trajectory=predicted_trajectory,
            confidence=confidence,
            behavior_class=behavior_class,
            time_horizon=self.prediction_horizon
        )
    
    def _estimate_lane_position(self, tracked_object: TrackedObject) -> Optional[int]:
        """Estimate which lane the object is currently in"""
        # This would normally use road structure information from map data
        # For now, we'll make a simplified estimation based on position relative to ego vehicle
        x_pos = tracked_object.state[0]  # Object's x position
        
        # Simplified lane estimation (in ego vehicle coordinate system)
        if abs(x_pos) < 1.5:  # Within center lane
            return 0
        elif x_pos < -1.5:  # To the left of ego
            return -1
        else:  # To the right of ego
            return 1
    
    def _estimate_intended_lane(self, tracked_object: TrackedObject) -> Optional[int]:
        """Estimate which lane the object intends to move to"""
        # Determine if object is changing lanes based on lateral movement
        lateral_velocity = abs(tracked_object.state[4])  # absolute vy
        
        if lateral_velocity > self.lane_change_threshold:
            # If moving laterally, estimate intended lane based on direction
            if tracked_object.state[4] > 0:  # Moving right
                return 1
            else:  # Moving left
                return -1
        
        return None
    
    def _update_history(self, object_id: str, state: VehicleState):
        """Update the historical record for an object"""
        self.object_history[object_id].append(state)
    
    def _classify_behavior(self, object_id: str, current_state: VehicleState) -> str:
        """Classify the object's current behavior"""
        # Get historical data
        history = list(self.object_history[object_id])
        
        # Default to lane following for first detections
        if len(history) < 2:
            return "lane_following"
        
        # Check for lane change
        lateral_velocity = abs(current_state.velocity[1])  # vy component
        if lateral_velocity > self.lane_change_threshold:
            if current_state.velocity[1] > 0:
                return "lane_changing_right"
            else:
                return "lane_changing_left"
        
        # Check for stopping behavior
        longitudinal_deceleration = current_state.acceleration[0]  # ax
        if longitudinal_deceleration < self.stopping_threshold:
            return "stopping"
        
        # Check for accelerating behavior
        if current_state.velocity[0] > self.speeding_threshold:
            return "highway_driving"
        
        # Check for turning behavior (significant lateral acceleration)
        if abs(current_state.acceleration[1]) > 1.0:
            if current_state.acceleration[1] > 0:
                return "turning_right"
            else:
                return "turning_left"
        
        # Default to lane following
        return "lane_following"
    
    def _generate_trajectory(self, vehicle_state: VehicleState, behavior_class: str) -> np.ndarray:
        """Generate predicted future trajectory based on behavior class"""
        trajectory = np.zeros((self.num_steps, 3), dtype=np.float32)
        
        # Start from current position
        current_pos = vehicle_state.position.copy()
        current_vel = vehicle_state.velocity.copy()
        current_acc = vehicle_state.acceleration.copy()
        
        # Different trajectory models based on behavior class
        if behavior_class == "lane_following":
            # Constant velocity model
            for i in range(self.num_steps):
                dt = (i + 1) * self.time_step
                # Update position: s = s0 + v*t + 0.5*a*t^2
                trajectory[i] = current_pos + current_vel * dt + 0.5 * current_acc * dt**2
        
        elif behavior_class in ["lane_changing_left", "lane_changing_right"]:
            # Model with lateral movement for lane changes
            for i in range(self.num_steps):
                dt = (i + 1) * self.time_step
                # Longitudinal movement (with some deceleration during lane change)
                longitudinal_pos = current_pos[0] + current_vel[0] * dt + 0.5 * current_acc[0] * dt**2
                # Lateral movement (more aggressive during lane change)
                lateral_pos = current_pos[1] + current_vel[1] * dt + 0.5 * current_acc[1] * dt**2
                # Vertical movement (minimal)
                vertical_pos = current_pos[2] + current_vel[2] * dt + 0.5 * current_acc[2] * dt**2
                
                trajectory[i] = np.array([longitudinal_pos, lateral_pos, vertical_pos])
        
        elif behavior_class == "stopping":
            # Model with deceleration
            for i in range(self.num_steps):
                dt = (i + 1) * self.time_step
                # Apply stronger deceleration model
                adjusted_acc = current_acc.copy()
                adjusted_acc[0] = max(adjusted_acc[0], self.stopping_threshold)  # Don't go below stopping threshold
                trajectory[i] = current_pos + current_vel * dt + 0.5 * adjusted_acc * dt**2
        
        elif behavior_class in ["turning_right", "turning_left"]:
            # Model with turning motion (circular arc approximation)
            for i in range(self.num_steps):
                dt = (i + 1) * self.time_step
                # For simplicity, use constant turn rate model
                turn_rate = current_acc[1] / max(current_vel[0], 0.1)  # Avoid division by zero
                turn_radius = current_vel[0] / max(abs(turn_rate), 0.01) if turn_rate != 0 else float('inf')
                
                if turn_rate > 0:  # Turning right
                    dx = current_vel[0] * dt
                    dy = -turn_radius * (1 - math.cos(turn_rate * dt))
                else:  # Turning left
                    dx = current_vel[0] * dt
                    dy = turn_radius * (1 - math.cos(turn_rate * dt))
                
                trajectory[i] = np.array([
                    current_pos[0] + dx,
                    current_pos[1] + dy,
                    current_pos[2]  # Assume minimal vertical change during turns
                ])
        
        elif behavior_class == "highway_driving":
            # High-speed, more predictable behavior
            for i in range(self.num_steps):
                dt = (i + 1) * self.time_step
                # More conservative acceleration changes at high speed
                conservative_acc = current_acc * 0.7  # Reduce acceleration impact at high speed
                trajectory[i] = current_pos + current_vel * dt + 0.5 * conservative_acc * dt**2
        
        else:
            # Default to constant velocity model
            for i in range(self.num_steps):
                dt = (i + 1) * self.time_step
                trajectory[i] = current_pos + current_vel * dt + 0.5 * current_acc * dt**2
        
        return trajectory
    
    def _calculate_prediction_confidence(self, tracked_object: TrackedObject, behavior_class: str) -> float:
        """Calculate confidence in the behavior prediction"""
        # Base confidence on tracking quality
        base_confidence = tracked_object.confidence
        
        # Adjust based on historical consistency
        history = list(self.object_history[tracked_object.id])
        consistency_factor = 1.0
        
        if len(history) > 2:
            # Calculate consistency of behavior classification over time
            recent_classifications = [
                self._classify_behavior(tracked_object.id, state) 
                for state in history[-3:]  # Last 3 states
            ]
            
            # If behavior is consistent, increase confidence
            if len(set(recent_classifications)) == 1:
                consistency_factor = 1.2  # Increase confidence for consistent behavior
        
        # Adjust based on object type (vehicles more predictable than pedestrians)
        type_factor = 1.0
        if tracked_object.class_name in ['car', 'truck', 'bus']:
            type_factor = 1.1  # Vehicles are more predictable
        elif tracked_object.class_name == 'person':
            type_factor = 0.7  # Pedestrians are less predictable
        elif tracked_object.class_name == 'bicycle':
            type_factor = 0.8  # Cyclists moderately predictable
        
        # Final confidence with bounds
        final_confidence = min(1.0, base_confidence * consistency_factor * type_factor)
        
        return max(0.1, final_confidence)  # Minimum confidence of 0.1


class PedestrianBehaviorPredictor:
    """Specialized predictor for pedestrian behavior"""

    def __init__(self):
        self.crosswalk_proximity_threshold = 10.0  # meters
        self.speed_threshold_standing = 0.5  # m/s

    def predict(self, tracked_object: TrackedObject) -> PredictionResult:
        """Predict pedestrian behavior with specialized models"""
        current_pos = tracked_object.state[:3]  # x, y, z
        current_vel = tracked_object.state[3:6]  # vx, vy, vz
        current_acc = tracked_object.state[6:9]  # ax, ay, az

        # Calculate speed
        speed = np.linalg.norm(current_vel)

        # Determine behavior based on context
        if speed < self.speed_threshold_standing:
            # Pedestrian is standing still
            behavior_class = "standing"
            # Predict they'll remain stationary with some probability of movement
            predicted_trajectory = self._predict_stationary_trajectory(current_pos, 5.0)
        elif self._near_crosswalk(current_pos):
            # Near crosswalk - more cautious prediction
            behavior_class = "crossing_preparation" if speed < 2.0 else "crossing"
            predicted_trajectory = self._predict_crossing_trajectory(
                current_pos, current_vel, current_acc
            )
        else:
            # Walking behavior
            behavior_class = "walking"
            predicted_trajectory = self._predict_walking_trajectory(
                current_pos, current_vel, current_acc
            )

        # Calculate confidence based on pedestrian-specific factors
        confidence = self._calculate_pedestrian_confidence(tracked_object, behavior_class)

        return PredictionResult(
            predicted_trajectory=predicted_trajectory,
            confidence=confidence,
            behavior_class=behavior_class,
            time_horizon=5.0
        )

    def _near_crosswalk(self, position: np.ndarray) -> bool:
        """Check if pedestrian is near a crosswalk (simplified for this implementation)"""
        # In a real implementation, this would use map data
        # For now, return False as a simplified approach
        return False

    def _predict_stationary_trajectory(self, position: np.ndarray, duration: float) -> np.ndarray:
        """Predict trajectory for stationary pedestrian"""
        num_steps = 10
        dt = duration / num_steps
        trajectory = np.zeros((num_steps, 3), dtype=np.float32)

        # Predict staying in approximately the same location with some random movement
        for i in range(num_steps):
            # Add slight random movement to represent natural fidgeting
            random_offset = np.random.normal(0, 0.1, size=3)  # 10cm std dev
            trajectory[i] = position + random_offset

        return trajectory

    def _predict_crossing_trajectory(self, position: np.ndarray, velocity: np.ndarray,
                                   acceleration: np.ndarray) -> np.ndarray:
        """Predict trajectory for pedestrian crossing road"""
        num_steps = 10
        dt = 0.5  # 0.5 second steps
        trajectory = np.zeros((num_steps, 3), dtype=np.float32)

        current_pos = position.copy()

        for i in range(num_steps):
            dt_step = (i + 1) * dt
            # For crossing, assume more predictable movement along crosswalk
            # Simplified model: constant velocity with occasional direction changes
            adjusted_vel = velocity * (0.8 + 0.4 * np.random.random())  # 80-120% of current speed
            trajectory[i] = current_pos + adjusted_vel * dt_step

        return trajectory

    def _predict_walking_trajectory(self, position: np.ndarray, velocity: np.ndarray,
                                  acceleration: np.ndarray) -> np.ndarray:
        """Predict trajectory for pedestrian walking"""
        num_steps = 10
        dt = 0.5
        trajectory = np.zeros((num_steps, 3), dtype=np.float32)

        current_pos = position.copy()

        for i in range(num_steps):
            dt_step = (i + 1) * dt
            # Walking is more predictable than random movement
            # Add some natural variation in walking pattern
            speed_factor = 0.9 + 0.2 * np.random.random()  # 90-110% of current speed
            adjusted_vel = velocity * speed_factor
            trajectory[i] = current_pos + adjusted_vel * dt_step

        return trajectory

    def _calculate_pedestrian_confidence(self, tracked_object: TrackedObject,
                                       behavior_class: str) -> float:
        """Calculate confidence specific to pedestrian prediction"""
        base_confidence = tracked_object.confidence

        # Pedestrians are inherently less predictable than vehicles
        if behavior_class == "standing":
            confidence = base_confidence * 1.1  # More predictable when standing
        elif behavior_class == "walking":
            confidence = base_confidence * 0.8  # Less predictable than vehicles
        else:  # crossing
            confidence = base_confidence * 0.7  # Even less predictable due to complex behavior

        return max(0.1, min(1.0, confidence))


class CyclistBehaviorPredictor:
    """Specialized predictor for cyclist behavior"""

    def __init__(self):
        self.bike_lane_width = 3.0  # meters
        self.speed_threshold_biking = 1.0  # m/s

    def predict(self, tracked_object: TrackedObject) -> PredictionResult:
        """Predict cyclist behavior with specialized models"""
        current_pos = tracked_object.state[:3]
        current_vel = tracked_object.state[3:6]
        current_acc = tracked_object.state[6:9]

        speed = np.linalg.norm(current_vel)

        if speed < self.speed_threshold_biking:
            behavior_class = "stopped"
            predicted_trajectory = self._predict_stationary_trajectory(current_pos, 3.0)
        else:
            # Cyclists can be more predictable than pedestrians but less than cars
            behavior_class = "cycling"
            predicted_trajectory = self._predict_cycling_trajectory(
                current_pos, current_vel, current_acc
            )

        confidence = self._calculate_cyclist_confidence(tracked_object, behavior_class)

        return PredictionResult(
            predicted_trajectory=predicted_trajectory,
            confidence=confidence,
            behavior_class=behavior_class,
            time_horizon=5.0
        )

    def _predict_stationary_trajectory(self, position: np.ndarray, duration: float) -> np.ndarray:
        """Predict trajectory for stationary cyclist"""
        num_steps = 6
        dt = duration / num_steps
        trajectory = np.zeros((num_steps, 3), dtype=np.float32)

        for i in range(num_steps):
            # Cyclists may have slightly more movement than pedestrians when stopped
            random_offset = np.random.normal(0, 0.2, size=3)  # 20cm std dev
            trajectory[i] = position + random_offset

        return trajectory

    def _predict_cycling_trajectory(self, position: np.ndarray, velocity: np.ndarray,
                                  acceleration: np.ndarray) -> np.ndarray:
        """Predict trajectory for cycling"""
        num_steps = 10
        dt = 0.5
        trajectory = np.zeros((num_steps, 3), dtype=np.float32)

        current_pos = position.copy()
        current_vel = velocity.copy()

        for i in range(num_steps):
            dt_step = (i + 1) * dt
            # Cyclists follow road geometry more than pedestrians but less constrained than cars
            # Add moderate variation for natural cycling behavior
            speed_factor = 0.85 + 0.3 * np.random.random()  # 85-115% of current speed
            adjusted_vel = current_vel * speed_factor
            trajectory[i] = current_pos + adjusted_vel * dt_step

        return trajectory

    def _calculate_cyclist_confidence(self, tracked_object: TrackedObject,
                                   behavior_class: str) -> float:
        """Calculate confidence specific to cyclist prediction"""
        base_confidence = tracked_object.confidence

        # Cyclists are moderately predictable
        if behavior_class == "stopped":
            confidence = base_confidence * 1.0  # As predictable as other stopped objects
        else:  # cycling
            confidence = base_confidence * 0.85  # More predictable than pedestrians, less than cars

        return max(0.1, min(1.0, confidence))


class SocialInteractionModel:
    """Model predicting interactions between road participants"""

    def __init__(self):
        # Define interaction patterns
        self.interaction_patterns = {
            'intersection_negotiation': {
                'conditions': ['approaching_intersection', 'multiple_vehicles'],
                'time_to_interaction': 3.0
            },
            'lane_merge_negotiation': {
                'conditions': ['similar_speed', 'lateral_convergence'],
                'time_to_interaction': 2.0
            },
            'following_behavior': {
                'conditions': ['same_lane', 'similar_speed'],
                'time_to_interaction': 5.0
            }
        }

    def predict(self, obj1: TrackedObject, obj2: TrackedObject) -> str:
        """Predict type of interaction between two objects"""
        # Determine relative positions and movements
        rel_pos = obj2.state[:3] - obj1.state[:3]  # Relative position
        rel_vel = obj2.state[3:6] - obj1.state[3:6]  # Relative velocity

        # Check for various interaction types
        if self._is_approaching_intersection(obj1, obj2):
            return "intersection_negotiation"
        elif self._is_lane_merge_scenario(obj1, obj2):
            return "lane_merge_negotiation"
        elif self._is_following_scenario(obj1, obj2):
            return "following"
        else:
            return "none"

    def _is_approaching_intersection(self, obj1: TrackedObject, obj2: TrackedObject) -> bool:
        """Check if objects are approaching an intersection"""
        # Implementation would use map data and predicted paths
        # For now, use relative position and heading as a simplified approach
        # In a real implementation, this would use HD map data
        return False  # Simplified for this example

    def _is_lane_merge_scenario(self, obj1: TrackedObject, obj2: TrackedObject) -> bool:
        """Check if objects are in a lane merge scenario"""
        # Check if objects are at similar speeds and converging laterally
        speed_diff = abs(obj1.state[3] - obj2.state[3])  # Difference in x-velocity
        lateral_distance = abs(obj1.state[1] - obj2.state[1])  # Difference in y-position
        lateral_velocity_diff = abs(obj1.state[4] - obj2.state[4])  # Difference in y-velocity

        return (speed_diff < 2.0 and lateral_distance < 10.0 and lateral_velocity_diff > 0.5)

    def _is_following_scenario(self, obj1: TrackedObject, obj2: TrackedObject) -> bool:
        """Check if one object is following another"""
        longitudinal_distance = obj2.state[0] - obj1.state[0]  # Distance from obj1 to obj2
        longitudinal_velocity_diff = obj1.state[3] - obj2.state[3]  # obj1 faster than obj2

        return (0 < longitudinal_distance < 50 and longitudinal_velocity_diff > 0)


class EnhancedPredictionSystem:
    """
    Enhanced prediction system that integrates with the tracking system
    to provide proactive planning capabilities
    """

    def __init__(self):
        self.behavior_predictor = BehaviorPredictor()
        self.pedestrian_predictor = PedestrianBehaviorPredictor()
        self.cyclist_predictor = CyclistBehaviorPredictor()
        self.social_interaction_model = SocialInteractionModel()
        self.prediction_cache = {}  # Cache for efficiency
        self.last_prediction_time = {}  # Track when we last predicted for each object

    def predict_all_behaviors(self, tracked_objects: Dict[str, TrackedObject]) -> Dict[str, PredictionResult]:
        """Predict behaviors for all tracked objects with object-type specific models"""
        predictions = {}

        for obj_id, tracked_obj in tracked_objects.items():
            # Use specialized predictor based on object class
            if tracked_obj.class_name == 'person':
                prediction = self.pedestrian_predictor.predict(tracked_obj)
            elif tracked_obj.class_name == 'bicycle':
                prediction = self.cyclist_predictor.predict(tracked_obj)
            elif tracked_obj.class_name in ['car', 'truck', 'bus']:
                prediction = self.behavior_predictor.predict_behavior(tracked_obj)
            else:
                # Default to general predictor for unknown object types
                prediction = self.behavior_predictor.predict_behavior(tracked_obj)

            predictions[obj_id] = prediction

        return predictions

    def predict_social_interactions(self, tracked_objects: Dict[str, TrackedObject]) -> Dict[str, List[str]]:
        """Predict interactions between multiple tracked objects"""
        interactions = {}

        # Analyze potential interactions between objects
        for obj_id_1, obj_1 in tracked_objects.items():
            nearby_objects = self._get_nearby_objects(obj_1, tracked_objects, distance_threshold=50.0)

            for obj_id_2 in nearby_objects:
                interaction_type = self.social_interaction_model.predict(
                    obj_1, tracked_objects[obj_id_2]
                )

                if interaction_type != "none":
                    if obj_id_1 not in interactions:
                        interactions[obj_id_1] = []
                    interactions[obj_id_1].append(f"{obj_id_2}:{interaction_type}")

        return interactions

    def _get_nearby_objects(self, reference_obj: TrackedObject,
                          all_objects: Dict[str, TrackedObject],
                          distance_threshold: float) -> List[str]:
        """Get objects within a certain distance of the reference object"""
        nearby = []
        ref_pos = reference_obj.state[:3]  # x, y, z

        for obj_id, obj in all_objects.items():
            if obj_id == reference_obj.id:
                continue  # Skip the reference object itself

            obj_pos = obj.state[:3]
            distance = np.linalg.norm(ref_pos - obj_pos)

            if distance <= distance_threshold:
                nearby.append(obj_id)

        return nearby
    
    def get_risk_assessment(self, predictions: Dict[str, PredictionResult], 
                           ego_state: Dict[str, float]) -> Dict[str, float]:
        """Assess risk based on predicted behaviors"""
        risk_metrics = {
            'collision_risk': 0.0,
            'lane_change_interference': 0.0,
            'stopping_distance_risk': 0.0,
            'total_threat_score': 0.0
        }
        
        for obj_id, prediction in predictions.items():
            # Calculate risk based on predicted trajectory and behavior
            threat_score = self._calculate_threat_score(prediction, ego_state)
            risk_metrics['total_threat_score'] += threat_score
            
            # Categorize risk by behavior type
            if 'lane_changing' in prediction.behavior_class:
                risk_metrics['lane_change_interference'] = max(
                    risk_metrics['lane_change_interference'], 
                    threat_score
                )
            elif prediction.behavior_class == 'stopping':
                risk_metrics['stopping_distance_risk'] = max(
                    risk_metrics['stopping_distance_risk'], 
                    threat_score
                )
        
        # Normalize risk scores
        if len(predictions) > 0:
            risk_metrics['total_threat_score'] /= len(predictions)
        
        # Calculate collision risk based on trajectory overlap
        risk_metrics['collision_risk'] = self._calculate_collision_risk(predictions, ego_state)
        
        return risk_metrics
    
    def _calculate_threat_score(self, prediction: PredictionResult, ego_state: Dict[str, float]) -> float:
        """Calculate threat score for a single predicted object"""
        # Higher threat for high-confidence, short-term predictions
        base_threat = prediction.confidence
        
        # Increase threat for aggressive behaviors
        behavior_multipliers = {
            'stopping': 1.2,
            'lane_changing_left': 1.5,
            'lane_changing_right': 1.5,
            'turning_right': 1.0,
            'turning_left': 1.0,
            'highway_driving': 0.8,
            'lane_following': 0.6
        }
        
        multiplier = behavior_multipliers.get(prediction.behavior_class, 1.0)
        threat_score = base_threat * multiplier
        
        return min(1.0, threat_score)
    
    def _calculate_collision_risk(self, predictions: Dict[str, PredictionResult], 
                                 ego_state: Dict[str, float]) -> float:
        """Calculate collision risk based on trajectory predictions"""
        max_collision_risk = 0.0
        
        # This is a simplified collision risk calculation
        # In a real system, this would involve more complex trajectory intersection analysis
        for obj_id, prediction in predictions.items():
            # Check if predicted trajectory comes close to ego path
            for future_pos in prediction.predicted_trajectory:
                # Calculate distance to ego's predicted path
                # For simplicity, just use Euclidean distance to ego position
                ego_pos = np.array([ego_state.get('x', 0), ego_state.get('y', 0), ego_state.get('z', 0)])
                distance = np.linalg.norm(future_pos - ego_pos)
                
                # Higher risk for closer objects
                if distance < 5.0:  # Within 5m
                    risk_contribution = (5.0 - distance) / 5.0 * prediction.confidence
                    max_collision_risk = max(max_collision_risk, risk_contribution)
        
        return max_collision_risk


def create_behavior_prediction_system() -> EnhancedPredictionSystem:
    """Factory function to create behavior prediction system"""
    return EnhancedPredictionSystem()


if __name__ == "__main__":
    print("Behavior Prediction System for Sunnypilot - Testing")
    print("=" * 55)
    
    # Create mock tracked object for testing
    class MockTrackedObject:
        def __init__(self, state, class_name, confidence):
            self.state = state  # [x, y, z, vx, vy, vz, ax, ay, az]
            self.class_name = class_name
            self.confidence = confidence
            self.id = "mock_obj_1"
    
    # Create predictor
    predictor = BehaviorPredictor()
    
    # Test with a mock tracked object (moving straight ahead)
    mock_object = MockTrackedObject(
        state=np.array([10.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32),
        class_name='car',
        confidence=0.9
    )
    
    # Predict behavior
    prediction = predictor.predict_behavior(mock_object)
    
    print(f"Object class: {mock_object.class_name}")
    print(f"Predicted behavior: {prediction.behavior_class}")
    print(f"Prediction confidence: {prediction.confidence:.2f}")
    print(f"Predicted trajectory shape: {prediction.predicted_trajectory.shape}")
    print(f"First few predicted positions: {prediction.predicted_trajectory[:3]}")
    
    # Test with vehicle changing lanes
    lane_changing_object = MockTrackedObject(
        state=np.array([15.0, 0.0, 0.0, 12.0, 2.0, 0.0, 0.0, 0.3, 0.0], dtype=np.float32),
        class_name='car',
        confidence=0.85
    )
    
    lane_change_prediction = predictor.predict_behavior(lane_changing_object)
    
    print(f"\nLane-changing object behavior: {lane_change_prediction.behavior_class}")
    print(f"Prediction confidence: {lane_change_prediction.confidence:.2f}")
    
    # Test with vehicle stopping
    stopping_object = MockTrackedObject(
        state=np.array([20.0, 0.0, 0.0, 8.0, 0.0, 0.0, -2.0, 0.0, 0.0], dtype=np.float32),  # Decelerating
        class_name='car',
        confidence=0.9
    )
    
    stopping_prediction = predictor.predict_behavior(stopping_object)
    
    print(f"\nStopping object behavior: {stopping_prediction.behavior_class}")
    print(f"Prediction confidence: {stopping_prediction.confidence:.2f}")
    
    print("\nBehavior prediction system ready for integration with planning system")