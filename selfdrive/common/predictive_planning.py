"""
Predictive Planning System for Sunnypilot
Uses enhanced camera fusion outputs to predict future states and plan trajectories
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import math
from dataclasses import dataclass
from enum import Enum
import time


class ManeuverType(Enum):
    """Types of maneuvers the planner can execute"""
    LANE_FOLLOW = "lane_follow"
    LANE_CHANGE_LEFT = "lane_change_left"
    LANE_CHANGE_RIGHT = "lane_change_right"
    STOP = "stop"
    YIELD = "yield"
    ADAPTIVE_CRUISE = "adaptive_cruise"


@dataclass
class PredictionState:
    """State prediction for a tracked object"""
    id: int
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    acceleration: np.ndarray  # [ax, ay, az]
    heading: float  # in radians
    timestamp: float
    predicted_trajectory: np.ndarray  # shape: [time_steps, 3]
    confidence: float  # 0.0 to 1.0


@dataclass
class PlannedTrajectory:
    """Planned trajectory for the ego vehicle"""
    positions: np.ndarray  # shape: [time_steps, 3] - [x, y, z]
    velocities: np.ndarray  # shape: [time_steps, 3] - [vx, vy, vz]
    headings: np.ndarray    # shape: [time_steps] - heading in radians
    accelerations: np.ndarray # shape: [time_steps, 3] - [ax, ay, az]
    time_stamps: np.ndarray   # shape: [time_steps]
    maneuver_type: ManeuverType
    cost: float
    valid: bool


class TrajectoryPredictor:
    """Predicts future trajectories based on current state and environmental context"""
    
    def __init__(self, time_horizon: float = 5.0, time_step: float = 0.1):
        self.time_horizon = time_horizon  # seconds
        self.time_step = time_step        # seconds
        self.num_steps = int(time_horizon / time_step)
        
        # Motion model parameters
        self.process_noise = np.diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])  # [pos, vel]
        self.measurement_noise = np.diag([0.05, 0.05, 0.05])  # [pos]
    
    def predict_constant_velocity(self, 
                                position: np.ndarray, 
                                velocity: np.ndarray, 
                                heading: float) -> np.ndarray:
        """Predict trajectory assuming constant velocity motion"""
        time_points = np.arange(0, self.time_horizon, self.time_step)
        trajectory = np.zeros((len(time_points), 3))
        
        for i, t in enumerate(time_points):
            # Apply constant velocity model
            pos_offset = velocity * t
            trajectory[i, :2] = position[:2] + pos_offset[:2]  # Update x, y
            trajectory[i, 2] = position[2]  # Keep z constant for now
        
        return trajectory
    
    def predict_constant_acceleration(self,
                                    position: np.ndarray,
                                    velocity: np.ndarray,
                                    acceleration: np.ndarray,
                                    heading: float) -> np.ndarray:
        """Predict trajectory with constant acceleration"""
        time_points = np.arange(0, self.time_horizon, self.time_step)
        trajectory = np.zeros((len(time_points), 3))
        
        for i, t in enumerate(time_points):
            # Apply kinematic equations: s = ut + 0.5at^2
            pos_offset = velocity * t + 0.5 * acceleration * t**2
            trajectory[i, :2] = position[:2] + pos_offset[:2]
            trajectory[i, 2] = position[2]  # Keep z constant
        
        return trajectory
    
    def predict_with_heading_change(self,
                                  position: np.ndarray,
                                  velocity: np.ndarray,
                                  heading_rate: float) -> np.ndarray:
        """Predict trajectory accounting for heading changes"""
        time_points = np.arange(0, self.time_horizon, self.time_step)
        trajectory = np.zeros((len(time_points), 3))
        
        current_heading = math.atan2(velocity[1], velocity[0])  # Initial heading from velocity
        
        trajectory[0, :2] = position[:2]
        
        for i in range(1, len(time_points)):
            dt = self.time_step
            # Update heading
            new_heading = current_heading + heading_rate * dt
            
            # Move in the direction of current heading
            dx = velocity[0] * dt * math.cos(new_heading)
            dy = velocity[0] * dt * math.sin(new_heading)
            
            trajectory[i, 0] = trajectory[i-1, 0] + dx
            trajectory[i, 1] = trajectory[i-1, 1] + dy
            trajectory[i, 2] = position[2]
            
            current_heading = new_heading
        
        return trajectory


class BehaviorPredictor:
    """Predicts behavior of other road users based on their current state and context"""
    
    def __init__(self):
        self.max_prediction_horizon = 5.0  # seconds
        self.trajectory_predictor = TrajectoryPredictor()
    
    def predict_vehicle_behavior(self, prediction_state: PredictionState, 
                               context: Dict[str, Any]) -> PredictionState:
        """Predict behavior of a tracked vehicle"""
        # Determine the likely behavior based on current context
        behavior_type = self._classify_behavior_type(prediction_state, context)
        
        if behavior_type == "constant_velocity":
            predicted_trajectory = self.trajectory_predictor.predict_constant_velocity(
                prediction_state.position,
                prediction_state.velocity,
                prediction_state.heading
            )
        elif behavior_type == "accelerating":
            # Estimate acceleration from velocity change or context
            acceleration = self._estimate_acceleration(prediction_state, context)
            predicted_trajectory = self.trajectory_predictor.predict_constant_acceleration(
                prediction_state.position,
                prediction_state.velocity,
                acceleration,
                prediction_state.heading
            )
        elif behavior_type == "lane_changing":
            # Predict lane change trajectory
            heading_rate = self._estimate_heading_rate(prediction_state, context)
            predicted_trajectory = self.trajectory_predictor.predict_with_heading_change(
                prediction_state.position,
                prediction_state.velocity,
                heading_rate
            )
        elif behavior_type == "stopping":
            # Predict deceleration to stop
            predicted_trajectory = self._predict_stopping_trajectory(prediction_state)
        else:
            # Default to constant velocity
            predicted_trajectory = self.trajectory_predictor.predict_constant_velocity(
                prediction_state.position,
                prediction_state.velocity,
                prediction_state.heading
            )
        
        # Update the prediction state
        updated_state = PredictionState(
            id=prediction_state.id,
            position=prediction_state.position.copy(),
            velocity=prediction_state.velocity.copy(),
            acceleration=prediction_state.acceleration.copy(),
            heading=prediction_state.heading,
            timestamp=prediction_state.timestamp,
            predicted_trajectory=predicted_trajectory,
            confidence=prediction_state.confidence
        )
        
        return updated_state
    
    def _classify_behavior_type(self, prediction_state: PredictionState, 
                              context: Dict[str, Any]) -> str:
        """Classify the likely behavior type for a tracked object"""
        # Analyze the tracked object's characteristics and context
        v_ego = context.get('ego_velocity', np.array([0.0, 0.0, 0.0]))
        ego_position = context.get('ego_position', np.array([0.0, 0.0, 0.0]))
        
        # Calculate relative positions and velocities
        relative_pos = prediction_state.position - ego_position
        relative_vel = prediction_state.velocity - v_ego
        
        distance = np.linalg.norm(relative_pos[:2])
        relative_speed = np.linalg.norm(relative_vel[:2])
        
        # Simple classification rules (in a real system, this would use ML)
        if distance < 5.0 and relative_speed < 2.0:
            # Close and slow relative motion - likely stopping
            return "stopping"
        elif abs(prediction_state.velocity[1]) > 1.0:  # Significant lateral velocity
            # Significant lateral movement - likely lane changing
            return "lane_changing"
        elif prediction_state.acceleration[0] > 1.0:
            # Significant forward acceleration
            return "accelerating"
        else:
            # Default to constant velocity
            return "constant_velocity"
    
    def _estimate_acceleration(self, prediction_state: PredictionState, 
                             context: Dict[str, Any]) -> np.ndarray:
        """Estimate acceleration for the tracked object"""
        # In a real system, this would use historical tracking data
        # For now, return a reasonable estimate
        return np.array([0.5, 0.0, 0.0])  # Small forward acceleration
    
    def _estimate_heading_rate(self, prediction_state: PredictionState, 
                             context: Dict[str, Any]) -> float:
        """Estimate heading rate for lane change behavior"""
        # Simple estimate based on lateral velocity
        return prediction_state.velocity[1] * 0.1  # Roughly proportional to lateral velocity
    
    def _predict_stopping_trajectory(self, prediction_state: PredictionState) -> np.ndarray:
        """Predict trajectory for a vehicle coming to a stop"""
        time_points = np.arange(0, self.trajectory_predictor.time_horizon, 
                                self.trajectory_predictor.time_step)
        trajectory = np.zeros((len(time_points), 3))
        
        initial_pos = prediction_state.position
        initial_vel = prediction_state.velocity.copy()
        
        # Assume constant deceleration to stop
        decel_rate = max(1.0, np.linalg.norm(initial_vel[:2]) / 3.0)  # Come to stop in ~3s or faster
        
        trajectory[0, :2] = initial_pos[:2]
        
        for i in range(1, len(time_points)):
            dt = self.trajectory_predictor.time_step
            # Apply deceleration
            vel_magnitude = np.linalg.norm(initial_vel[:2])
            if vel_magnitude > 0:
                decel_vector = (initial_vel[:2] / vel_magnitude) * min(decel_rate * dt, vel_magnitude)
                initial_vel[:2] -= decel_vector
                new_pos = trajectory[i-1, :2] + initial_vel[:2] * dt
                trajectory[i, :2] = new_pos
            else:
                trajectory[i, :2] = trajectory[i-1, :2]  # Vehicle has stopped
            
            trajectory[i, 2] = initial_pos[2]  # Keep z constant
        
        return trajectory


class PathPlanner:
    """Plans optimal paths considering predicted environment"""
    
    def __init__(self, dt: float = 0.1, planning_horizon: float = 5.0):
        self.dt = dt
        self.planning_horizon = planning_horizon
        self.num_steps = int(planning_horizon / dt)
        
        # Planning constraints
        self.max_velocity = 35.0  # m/s (about 126 km/h)
        self.max_acceleration = 3.0  # m/s^2
        self.max_deceleration = -5.0  # m/s^2
        self.max_lateral_accel = 2.5  # m/s^2
    
    def plan_lane_follow(self, 
                        ego_state: Dict[str, Any],
                        predicted_environment: List[PredictionState]) -> PlannedTrajectory:
        """Plan trajectory for lane following"""
        # Extract ego state
        pos = ego_state['position']  # [x, y, z]
        vel = ego_state['velocity']  # [vx, vy, vz]
        heading = ego_state['heading']  # in radians
        
        # Initialize trajectory arrays
        positions = np.zeros((self.num_steps, 3))
        velocities = np.zeros((self.num_steps, 3))
        headings = np.zeros(self.num_steps)
        accelerations = np.zeros((self.num_steps, 3))
        
        # Set initial state
        positions[0] = pos.copy()
        velocities[0] = vel.copy()
        headings[0] = heading
        accelerations[0] = np.array([0.0, 0.0, 0.0])
        
        # Plan trajectory considering predicted obstacles
        for i in range(1, self.num_steps):
            dt = self.dt
            
            # Calculate desired motion based on lane following
            current_pos = positions[i-1]
            current_vel = velocities[i-1]
            
            # Maintain current lane (y position) with adaptive longitudinal control
            target_y = current_pos[1]  # Stay in current lane
            target_x = current_pos[0] + current_vel[0] * dt  # Continue forward
            
            # Adjust longitudinal velocity based on traffic
            desired_vx = self._adjust_velocity_for_traffic(
                current_pos, current_vel[0], predicted_environment, i*dt
            )
            
            # Update state
            positions[i, 0] = target_x
            positions[i, 1] = target_y
            positions[i, 2] = current_pos[2]  # Keep z constant for now
            
            velocities[i, 0] = desired_vx
            velocities[i, 1] = 0.0  # Minimal lateral velocity in lane follow
            velocities[i, 2] = 0.0
            
            # Calculate acceleration
            accelerations[i] = (velocities[i] - velocities[i-1]) / dt
            headings[i] = headings[i-1]  # Heading changes slowly in lane follow
        
        time_stamps = np.arange(0, self.planning_horizon, self.dt)
        
        # Calculate trajectory cost
        cost = self._calculate_trajectory_cost(positions, velocities, accelerations, 
                                             predicted_environment)
        
        return PlannedTrajectory(
            positions=positions,
            velocities=velocities,
            headings=headings,
            accelerations=accelerations,
            time_stamps=time_stamps,
            maneuver_type=ManeuverType.LANE_FOLLOW,
            cost=cost,
            valid=True
        )
    
    def plan_lane_change(self, 
                        ego_state: Dict[str, Any],
                        predicted_environment: List[PredictionState],
                        direction: str = 'left') -> PlannedTrajectory:
        """Plan trajectory for lane change"""
        # Extract ego state
        pos = ego_state['position']
        vel = ego_state['velocity']
        heading = ego_state['heading']
        
        # Initialize trajectory arrays
        positions = np.zeros((self.num_steps, 3))
        velocities = np.zeros((self.num_steps, 3))
        headings = np.zeros(self.num_steps)
        accelerations = np.zeros((self.num_steps, 3))
        
        # Set initial state
        positions[0] = pos.copy()
        velocities[0] = vel.copy()
        headings[0] = heading
        accelerations[0] = np.array([0.0, 0.0, 0.0])
        
        # Lane change parameters
        lane_width = 3.7  # meters
        change_duration = min(4.0, self.planning_horizon)  # seconds
        change_steps = int(change_duration / self.dt)
        
        # Plan lane change
        for i in range(1, self.num_steps):
            dt = self.dt
            current_pos = positions[i-1]
            current_vel = velocities[i-1]
            
            # Continue forward
            target_x = current_pos[0] + current_vel[0] * dt
            
            # Execute lateral movement for lane change during initial portion
            lateral_pos = current_pos[1]
            if i <= change_steps:
                # Calculate progress in lane change (0 to 1)
                progress = min(1.0, i / change_steps)
                
                # Use smooth transition (sigmoid-like)
                smooth_progress = 0.5 * (1 + math.tanh(3 * (2 * progress - 1)))
                
                if direction == 'left':
                    target_y = pos[1] + lane_width * smooth_progress
                else:  # right
                    target_y = pos[1] - lane_width * smooth_progress
            else:
                target_y = current_pos[1]  # Maintain new lane
            
            # Adjust velocity based on traffic in target lane
            desired_vx = self._adjust_velocity_for_traffic(
                np.array([target_x, target_y, current_pos[2]]), 
                current_vel[0], 
                predicted_environment, 
                i*dt
            )
            
            # Calculate desired lateral velocity for the transition
            if i <= change_steps:
                # Estimate lateral velocity needed for smooth transition
                if i > 0:
                    lateral_vel = (target_y - positions[i-1, 1]) / dt
                else:
                    lateral_vel = 0.0
            else:
                # Return to zero lateral velocity in new lane
                lateral_vel = 0.0
            
            # Update state
            positions[i, 0] = target_x
            positions[i, 1] = target_y
            positions[i, 2] = current_pos[2]
            
            velocities[i, 0] = desired_vx
            velocities[i, 1] = lateral_vel
            velocities[i, 2] = 0.0
            
            # Calculate acceleration
            accelerations[i] = (velocities[i] - velocities[i-1]) / dt
            headings[i] = math.atan2(velocities[i, 1], velocities[i, 0])
        
        time_stamps = np.arange(0, self.planning_horizon, self.dt)
        
        # Calculate trajectory cost (include lane change penalty)
        base_cost = self._calculate_trajectory_cost(positions, velocities, accelerations, 
                                                  predicted_environment)
        cost = base_cost * 1.1  # Slightly higher cost for lane change maneuver
        
        maneuver_type = ManeuverType.LANE_CHANGE_LEFT if direction == 'left' else ManeuverType.LANE_CHANGE_RIGHT
        
        return PlannedTrajectory(
            positions=positions,
            velocities=velocities,
            headings=headings,
            accelerations=accelerations,
            time_stamps=time_stamps,
            maneuver_type=maneuver_type,
            cost=cost,
            valid=True
        )
    
    def _adjust_velocity_for_traffic(self, 
                                   position: np.ndarray, 
                                   current_velocity: float,
                                   predictions: List[PredictionState],
                                   time_ahead: float) -> float:
        """Adjust desired velocity based on predicted traffic"""
        # Find nearby predicted vehicles
        safe_distance = current_velocity * 2.0  # 2-second rule
        min_safe_distance = 20.0  # Minimum distance
        
        adjusted_velocity = current_velocity
        
        for pred in predictions:
            # Get predicted position at time_ahead
            if time_ahead < len(pred.predicted_trajectory) * 0.1:
                idx = int(time_ahead / 0.1)
                if idx >= len(pred.predicted_trajectory):
                    idx = -1
                pred_pos = pred.predicted_trajectory[idx]
            else:
                pred_pos = pred.predicted_trajectory[-1]  # Use last prediction
            
            # Calculate distance to predicted vehicle
            dist_vec = pred_pos[:2] - position[:2]
            distance = np.linalg.norm(dist_vec)
            
            # If the predicted vehicle is ahead and close
            if (distance < 50.0 and  # Only consider vehicles within 50m
                dist_vec[0] > 0 and  # Only vehicles ahead (positive x direction)
                abs(dist_vec[1]) < 5.0):  # Similar lane (within 5m laterally)
                
                # Calculate time to collision at current speeds
                rel_vel = current_velocity - np.linalg.norm(pred.velocity[:2])
                if rel_vel > 0:  # We're approaching the vehicle
                    time_to_collision = distance / rel_vel if rel_vel > 0.1 else float('inf')
                    
                    if time_to_collision < 4.0:  # Less than 4 seconds
                        # Calculate safer velocity
                        safe_vel = max(0, np.linalg.norm(pred.velocity[:2]) - 2.0)  # Stay 2 m/s slower
                        adjusted_velocity = min(adjusted_velocity, safe_vel)
        
        # Apply physical constraints
        adjusted_velocity = max(0, min(adjusted_velocity, self.max_velocity))
        
        return adjusted_velocity
    
    def _calculate_trajectory_cost(self,
                                 positions: np.ndarray,
                                 velocities: np.ndarray,
                                 accelerations: np.ndarray,
                                 predictions: List[PredictionState]) -> float:
        """Calculate cost of the planned trajectory"""
        # Initialize cost components
        progress_cost = 0.0
        comfort_cost = 0.0
        safety_cost = 0.0
        
        # Cost for progress (reward forward movement)
        if len(positions) > 1:
            forward_movement = positions[-1, 0] - positions[0, 0]
            progress_cost = -forward_movement * 0.1  # Negative because we want to maximize progress
        
        # Cost for comfort (penalize high acceleration/jerk)
        for i in range(len(accelerations)):
            # Penalize high longitudinal and lateral acceleration
            longitudinal_acc = abs(accelerations[i, 0])
            lateral_acc = abs(accelerations[i, 1])
            
            # Apply asymmetric penalties (deceleration is more uncomfortable)
            if accelerations[i, 0] < 0:
                comfort_cost += longitudinal_acc ** 1.5  # Higher penalty for deceleration
            else:
                comfort_cost += longitudinal_acc ** 1.2
            
            comfort_cost += lateral_acc ** 1.5  # Lateral acceleration penalty
        
        # Cost for safety (penalize proximity to predicted vehicles)
        for i, pos in enumerate(positions):
            time_ahead = i * self.dt
            
            for pred in predictions:
                # Get predicted position at this time
                if time_ahead < len(pred.predicted_trajectory) * 0.1:
                    idx = int(time_ahead / 0.1)
                    if idx >= len(pred.predicted_trajectory):
                        idx = -1
                    pred_pos = pred.predicted_trajectory[idx]
                else:
                    pred_pos = pred.predicted_trajectory[-1]
                
                # Calculate distance
                dist_vec = pos[:2] - pred_pos[:2]
                distance = np.linalg.norm(dist_vec)
                
                if distance < 10.0:  # Consider only close objects
                    # Higher cost as distance decreases
                    safety_cost += 100.0 / (distance + 1.0)  # Asymptotic cost function
        
        # Combine costs with weights
        total_cost = (progress_cost * 0.5 +   # Progress is rewarded
                     comfort_cost * 1.0 +    # Comfort is important
                     safety_cost * 2.0)      # Safety has highest weight
        
        # Ensure positive cost
        total_cost = max(0.01, total_cost)
        
        return total_cost


class PredictivePlanner:
    """Main predictive planning system that integrates all components"""
    
    def __init__(self):
        self.behavior_predictor = BehaviorPredictor()
        self.path_planner = PathPlanner()
        self.last_planning_time = 0.0
        self.planning_frequency = 10.0  # Hz
    
    def plan(self, 
             ego_state: Dict[str, Any],
             tracked_objects: List[Dict[str, Any]],
             context: Dict[str, Any] = None) -> Tuple[Optional[PlannedTrajectory], List[PredictionState]]:
        """
        Plan trajectory considering predicted environment
        
        Args:
            ego_state: Current state of the ego vehicle
            tracked_objects: List of currently tracked objects from camera fusion
            context: Additional context (lane information, navigation, etc.)
            
        Returns:
            Tuple of (best_planned_trajectory, list_of_predicted_states)
        """
        current_time = time.time()
        
        # Limit planning frequency
        if current_time - self.last_planning_time < 1.0 / self.planning_frequency:
            return None, []
        
        self.last_planning_time = current_time
        
        if context is None:
            context = {}
        
        # Predict behaviors of tracked objects
        predicted_states = []
        
        for obj in tracked_objects:
            # Create initial prediction state from tracked object
            pred_state = PredictionState(
                id=obj.get('id', 0),
                position=np.array([obj.get('dRel', 0.0), obj.get('yRel', 0.0), 0.0]),  # [x, y, z]
                velocity=np.array([obj.get('vRel', 0.0), 0.0, 0.0]),  # [vx, vy, vz] - simplified
                acceleration=np.array([0.0, 0.0, 0.0]),
                heading=0.0,  # Simplified
                timestamp=current_time,
                predicted_trajectory=np.zeros((int(5.0/0.1), 3)),  # 5-second prediction at 0.1s steps
                confidence=obj.get('prob', 0.8)
            )
            
            # Predict behavior
            updated_pred = self.behavior_predictor.predict_vehicle_behavior(pred_state, context)
            predicted_states.append(updated_pred)
        
        # Plan trajectory
        best_trajectory = self._select_best_trajectory(ego_state, predicted_states)
        
        return best_trajectory, predicted_states
    
    def _select_best_trajectory(self, 
                               ego_state: Dict[str, Any], 
                               predicted_states: List[PredictionState]) -> Optional[PlannedTrajectory]:
        """Select the best trajectory from possible maneuvers"""
        # Get current ego state
        current_velocity = np.linalg.norm(ego_state['velocity'][:2])
        
        # Generate candidate trajectories
        trajectories = []
        
        # Lane follow trajectory
        lane_follow_traj = self.path_planner.plan_lane_follow(ego_state, predicted_states)
        trajectories.append(lane_follow_traj)
        
        # Check if lane changes are safe and beneficial
        if current_velocity > 5.0:  # Only consider lane changes at reasonable speeds
            # Left lane change
            left_change_traj = self.path_planner.plan_lane_change(
                ego_state, predicted_states, 'left'
            )
            if left_change_traj.valid:
                trajectories.append(left_change_traj)
            
            # Right lane change
            right_change_traj = self.path_planner.plan_lane_change(
                ego_state, predicted_states, 'right'
            )
            if right_change_traj.valid:
                trajectories.append(right_change_traj)
        
        # Select trajectory with lowest cost (best)
        if trajectories:
            best_trajectory = min(trajectories, key=lambda t: t.cost)
            return best_trajectory
        
        return None


def get_predictive_planner() -> PredictivePlanner:
    """Get the global predictive planner instance"""
    if not hasattr(get_predictive_planner, 'instance'):
        get_predictive_planner.instance = PredictivePlanner()
    return get_predictive_planner.instance


# Global instance for use across the system
predictive_planner = get_predictive_planner()


# Example usage and testing
if __name__ == "__main__":
    print("Testing Predictive Planning System...")
    
    # Create sample ego state
    ego_state = {
        'position': np.array([0.0, 0.0, 0.0]),  # [x, y, z]
        'velocity': np.array([15.0, 0.0, 0.0]),  # [vx, vy, vz] - moving forward at 15 m/s
        'heading': 0.0  # heading in radians
    }
    
    # Create sample tracked objects
    tracked_objects = [
        {
            'id': 1,
            'dRel': 20.0,  # 20m ahead
            'yRel': 0.5,   # 0.5m to the left (in our lane)
            'vRel': 12.0,  # moving at 12 m/s
            'prob': 0.9
        },
        {
            'id': 2,
            'dRel': 50.0,  # 50m ahead
            'yRel': 0.0,   # in our lane
            'vRel': 14.0,  # moving at 14 m/s
            'prob': 0.85
        },
        {
            'id': 3,
            'dRel': 15.0,  # 15m ahead
            'yRel': -3.7,  # in left lane
            'vRel': 10.0,  # moving slower
            'prob': 0.8
        }
    ]
    
    context = {
        'ego_velocity': ego_state['velocity'],
        'ego_position': ego_state['position'],
        'current_lane': 1,
        'num_lanes': 3
    }
    
    planner = get_predictive_planner()
    
    # Plan trajectory
    best_trajectory, predicted_states = planner.plan(ego_state, tracked_objects, context)
    
    if best_trajectory:
        print(f"Planning successful!")
        print(f"Selected maneuver: {best_trajectory.maneuver_type.value}")
        print(f"Trajectory cost: {best_trajectory.cost:.2f}")
        print(f"Trajectory duration: {len(best_trajectory.time_stamps) * 0.1:.1f}s")
        print(f"Final position: [{best_trajectory.positions[-1, 0]:.1f}, {best_trajectory.positions[-1, 1]:.1f}]")
    else:
        print("Planning failed to find valid trajectory")
    
    print(f"Predicted states for {len(predicted_states)} objects")
    
    print("\nPredictive planning system test completed!")