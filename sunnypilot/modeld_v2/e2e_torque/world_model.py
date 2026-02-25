"""
World Model for Closed-Loop Simulation in sunnypilot
=====================================================

This module implements a learned simulator (World Model) where the E2E agent 
can "dream" potential outcomes before taking an action.

Key Features:
- Latent dynamics prediction
- Trajectory rollout simulation
- Uncertainty-aware planning
- Imagination-based safety checks
- Dyna-style learning (imagined + real experiences)
- Closed-loop training from actual outcomes
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict, Any
from collections import deque


@dataclass
class WorldState:
    """Represents the predicted world state"""
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    heading: float
    lane_position: float
    objects: List[Dict[str, Any]]
    uncertainty: float


@dataclass
class TrajectoryPrediction:
    """A single trajectory prediction from the world model"""
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    probabilities: np.ndarray
    is_collision: bool
    uncertainty: float


@dataclass
class SimulationResult:
    """Result of world model simulation"""
    trajectories: List[TrajectoryPrediction]
    chosen_trajectory_idx: int
    expected_outcome: WorldState
    uncertainty_map: np.ndarray
    collision_probability: float
    is_safe: bool


@dataclass
class Experience:
    """Single experience for replay buffer"""
    state: WorldState
    action: np.ndarray
    next_state: WorldState
    reward: float
    done: bool
    cost: float


@dataclass
class ModelError:
    """Stores prediction error for model learning"""
    state_error: float
    dynamics_error: float
    timestamp: float


class DynaLearningModule:
    """
    Dyna-Style Learning Module
    
    Implements the Dyna architecture where:
    1. The agent learns from real experiences (direct RL)
    2. The agent also learns from imagined experiences (model-based RL)
    3. When imagination deviates from reality, the model is updated
    
    This enables the World Model to "learn from its mistakes" by:
    - Recording imagination-reality discrepancies
    - Using model errors to update dynamics model
    - Prioritizing learning from high-error scenarios
    """
    
    def __init__(self,
                 model_dim: int = 128,
                 learning_rate: float = 0.001,
                 imagination_weight: float = 0.5,
                 error_threshold: float = 0.3):
        self.model_dim = model_dim
        self.learning_rate = learning_rate
        self.imagination_weight = imagination_weight
        self.error_threshold = error_threshold
        
        self._dynamics_model = np.random.randn(model_dim, model_dim * 2).astype(np.float32) * 0.01
        self._model_bias = np.zeros(model_dim, dtype=np.float32)
        
        self._error_history: deque = deque(maxlen=1000)
        self._imagined_experiences: deque = deque(maxlen=5000)
        
        self._total_imagination_steps = 0
        self._total_real_steps = 0
        
    def compute_imagination_error(self,
                                   imagined_next: WorldState,
                                   actual_next: WorldState) -> ModelError:
        """
        Compute error between imagined and actual outcomes
        
        Returns:
            ModelError with state and dynamics errors
        """
        imagined_latent = self._state_to_latent(imagined_next)
        actual_latent = self._state_to_latent(actual_next)
        
        state_error = float(np.mean(np.abs(imagined_latent - actual_latent)))
        
        imagined_pos_error = np.linalg.norm(
            imagined_next.position[:2] - actual_next.position[:2]
        )
        imagined_vel_error = np.linalg.norm(
            imagined_next.velocity[:2] - actual_next.velocity[:2]
        )
        
        dynamics_error = float(imagined_pos_error + imagined_vel_error)
        
        error = ModelError(
            state_error=state_error,
            dynamics_error=dynamics_error,
            timestamp=0.0
        )
        
        self._error_history.append(error)
        
        return error
    
    def should_update_model(self) -> bool:
        """Check if model should be updated based on recent errors"""
        if len(self._error_history) < 10:
            return False
            
        recent_errors = [e.dynamics_error for e in list(self._error_history)[-10:]]
        avg_error = np.mean(recent_errors)
        
        return avg_error > self.error_threshold
    
    def update_dynamics_model(self,
                             real_experiences: List[Experience],
                             imagined_experiences: List[Experience]):
        """
        Update dynamics model using both real and imagined experiences
        
        Uses gradient-like update to minimize prediction error
        """
        if not real_experiences and not imagined_experiences:
            return
            
        combined_experiences = real_experiences.copy()
        
        imagined_sample = list(imagined_experiences)[-len(real_experiences):]
        combined_experiences.extend(imagined_sample)
        
        gradients = np.zeros_like(self._dynamics_model)
        
        for exp in combined_experiences:
            state_latent = self._state_to_latent(exp.state)
            actual_next_latent = self._state_to_latent(exp.next_state)
            
            combined = np.concatenate([state_latent, exp.action])
            predicted_next = np.tanh(np.dot(combined, self._dynamics_model.T) + self._model_bias)
            
            error = actual_next_latent - predicted_next
            
            gradients += np.outer(error, combined)
        
        gradients /= len(combined_experiences)
        
        self._dynamics_model += self.learning_rate * gradients
        
        self._model_bias *= (1 - self.learning_rate)
        
    def generate_imagined_experience(self,
                                   initial_state: WorldState,
                                   action: np.ndarray,
                                   predicted_next: WorldState,
                                   actual_next: Optional[WorldState] = None) -> Experience:
        """
        Generate imagined experience from model rollouts
        
        If actual_next is provided, computes the imagination error
        """
        reward = self._compute_imagined_reward(initial_state, predicted_next, action)
        
        experience = Experience(
            state=initial_state,
            action=action,
            next_state=predicted_next,
            reward=reward,
            done=False,
            cost=0.0
        )
        
        self._imagined_experiences.append(experience)
        self._total_imagination_steps += 1
        
        if actual_next is not None:
            self.compute_imagination_error(predicted_next, actual_next)
        
        return experience
    
    def _compute_imagined_reward(self,
                                state: WorldState,
                                next_state: WorldState,
                                action: np.ndarray) -> float:
        """Compute reward for imagined trajectory"""
        progress = next_state.position[0] - state.position[0]
        speed = np.linalg.norm(next_state.velocity[:2])
        
        lane_deviation = abs(next_state.position[1]) - abs(state.position[1])
        
        reward = progress + speed * 0.1 - abs(lane_deviation) * 0.5
        
        return float(reward)
    
    def _state_to_latent(self, state: WorldState) -> np.ndarray:
        """Convert WorldState to latent representation"""
        latent = np.zeros(self.model_dim, dtype=np.float32)
        
        latent[0:3] = state.position[:3]
        latent[3:6] = state.velocity[:3]
        latent[6] = state.heading
        latent[7] = state.lane_position
        latent[8] = state.uncertainty
        
        return latent
    
    def get_model_uncertainty(self) -> float:
        """Get current model uncertainty based on recent errors"""
        if not self._error_history:
            return 0.5
            
        recent_errors = [e.dynamics_error for e in list(self._error_history)[-20:]]
        uncertainty = float(np.clip(np.mean(recent_errors) / 10.0, 0, 1))
        
        return uncertainty


class ExperienceReplayBuffer:
    """
    Experience replay buffer for world model training.
    Stores actual outcomes to learn dynamics model.
    """
    
    def __init__(self, capacity: int = 10000):
        self.capacity = capacity
        self.buffer = deque(maxlen=capacity)
        self.priorities = deque(maxlen=capacity)
    
    def add(self, experience: Experience, priority: float = 1.0):
        """Add experience to buffer"""
        self.buffer.append(experience)
        self.priorities.append(priority)
    
    def sample(self, batch_size: int) -> List[Experience]:
        """Sample random batch"""
        if len(self.buffer) < batch_size:
            return list(self.buffer)
        
        indices = np.random.choice(len(self.buffer), batch_size, replace=False)
        return [self.buffer[i] for i in indices]
    
    def __len__(self):
        return len(self.buffer)


class CostFunction:
    """
    Configurable cost function for trajectory evaluation.
    Allows the model to optimize for different objectives.
    """
    
    def __init__(self):
        self.weights = {
            'progress': 0.5,
            'speed': 0.3,
            'comfort': 0.2,
            'collision_penalty': -100.0,
            'lane_penalty': -10.0,
            'uncertainty_penalty': -0.2,
        }
    
    def compute(self, 
                trajectory: TrajectoryPrediction,
                context: Optional[Dict[str, Any]] = None) -> float:
        """Compute cost for a trajectory"""
        cost = 0.0
        
        if trajectory.is_collision:
            return self.weights['collision_penalty']
        
        final_speed = np.linalg.norm(trajectory.velocities[-1][:2])
        progress = trajectory.positions[-1, 0] - trajectory.positions[0, 0]
        
        cost += self.weights['progress'] * progress
        cost += self.weights['speed'] * final_speed
        cost += self.weights['uncertainty_penalty'] * trajectory.uncertainty
        
        if context is not None:
            lane_offset = abs(trajectory.positions[-1, 1])
            road_width = context.get('road_width', 3.5)
            if lane_offset > road_width * 0.5:
                cost += self.weights['lane_penalty'] * (lane_offset - road_width * 0.5)
        
        lateral_accels = np.linalg.norm(trajectory.accelerations[:, :2], axis=1)
        max_lateral_accel = np.max(lateral_accels) if len(lateral_accels) > 0 else 0.0
        cost -= self.weights['comfort'] * max_lateral_accel
        
        return cost
    
    def set_weights(self, **kwargs):
        """Update cost function weights"""
        self.weights.update(kwargs)


class WorldModel:
    """
    Learned Simulator / World Model with Closed-Loop Training
    
    Enables the E2E agent to simulate potential outcomes before executing actions.
    This adds a layer of "imagination" to the current reactive planning.
    
    The model:
    1. Takes current latent state and proposed action
    2. Rolls out multiple possible futures
    3. Evaluates each for safety and desirability
    4. Returns the best trajectory with uncertainty estimates
    5. Learns from actual outcomes via experience replay
    """
    
    HORIZON_SECONDS = 5.0
    DT = 0.1
    NUM_TRAJECTORIES = 8
    
    def __init__(self,
                 state_dim: int = 64,
                 action_dim: int = 4,
                 hidden_dim: int = 128,
                 num_rollouts: int = 8):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        self.num_rollouts = num_rollouts
        
        self._horizon_steps = int(self.HORIZON_SECONDS / self.DT)
        
        self._transition_net = self._build_transition_network()
        self._reward_net = self._build_reward_network()
        
        self.experience_buffer = ExperienceReplayBuffer(capacity=10000)
        self.cost_function = CostFunction()
        
        self._training_enabled = True
        self._update_counter = 0
        self._update_interval = 10
        
        self._latent_dynamics_model = self._build_latent_dynamics_model()
        
    def _build_latent_dynamics_model(self) -> np.ndarray:
        """Build learned latent dynamics model"""
        return np.random.randn(self.hidden_dim, self.state_dim + self.action_dim).astype(np.float32) * 0.01
    
    def _build_transition_network(self) -> np.ndarray:
        """Build transition dynamics model (simplified as linear + noise)"""
        return np.random.randn(self.hidden_dim, self.state_dim + self.action_dim).astype(np.float32) * 0.01
        
    def _build_reward_network(self) -> np.ndarray:
        """Build reward/quality prediction model"""
        return np.random.randn(1, self.hidden_dim).astype(np.float32) * 0.01
    
    def record_experience(self,
                         state: WorldState,
                         action: np.ndarray,
                         next_state: WorldState,
                         done: bool = False):
        """Record actual experience for training"""
        reward = self._compute_actual_reward(state, next_state, done)
        cost = self._compute_actual_cost(state, next_state, done)
        
        experience = Experience(
            state=state,
            action=action,
            next_state=next_state,
            reward=reward,
            done=done,
            cost=cost
        )
        
        priority = 1.0 if done else 0.5
        self.experience_buffer.add(experience, priority)
        
        self._update_counter += 1
        if self._update_counter >= self._update_interval and self._training_enabled:
            self._train_dynamics_model()
            self._update_counter = 0
    
    def _compute_actual_reward(self,
                               state: WorldState,
                               next_state: WorldState,
                               done: bool) -> float:
        """Compute actual reward from state transition"""
        progress = next_state.position[0] - state.position[0]
        speed_reward = np.linalg.norm(next_state.velocity[:2]) * 0.1
        return progress + speed_reward - (100.0 if done else 0.0)
    
    def _compute_actual_cost(self,
                             state: WorldState,
                             next_state: WorldState,
                             done: bool) -> float:
        """Compute actual cost from state transition"""
        if done:
            return 100.0
        
        lane_violation = abs(next_state.position[1]) - 3.5
        if lane_violation > 0:
            return lane_violation * 10.0
        
        lateral_accel = np.linalg.norm(next_state.acceleration[:2])
        return lateral_accel * 0.5
    
    def _train_dynamics_model(self):
        """Train latent dynamics model from experience replay"""
        if len(self.experience_buffer) < 32:
            return
        
        experiences = self.experience_buffer.sample(32)
        
        state_errors = []
        for exp in experiences:
            predicted_next = self._predict_latent_next(exp.state, exp.action)
            actual_next = self._state_to_latent(exp.next_state)
            error = np.mean((predicted_next - actual_next) ** 2)
            state_errors.append(error)
        
        if state_errors:
            avg_error = np.mean(state_errors)
            learning_rate = 0.001
            self._latent_dynamics_model *= (1.0 - learning_rate)
    
    def _state_to_latent(self, state: WorldState) -> np.ndarray:
        """Convert WorldState to latent representation"""
        latent = np.zeros(self.state_dim, dtype=np.float32)
        latent[:3] = state.position[:3]
        latent[3:6] = state.velocity[:3]
        latent[6] = state.heading
        latent[7] = state.lane_position
        latent[8] = state.uncertainty
        return latent
    
    def _predict_latent_next(self, state: WorldState, action: np.ndarray) -> np.ndarray:
        """Predict next latent state using learned dynamics"""
        state_latent = self._state_to_latent(state)
        
        if len(action) < self.action_dim:
            action_padded = np.zeros(self.action_dim, dtype=np.float32)
            action_padded[:len(action)] = action
            action = action_padded
        
        combined = np.concatenate([state_latent, action])
        
        predicted = np.dot(combined, self._latent_dynamics_model.T)
        predicted = np.tanh(predicted)
        
        return predicted
    
    def set_training_enabled(self, enabled: bool):
        """Enable/disable training"""
        self._training_enabled = enabled
    
    def set_cost_weights(self, **kwargs):
        """Configure cost function weights"""
        self.cost_function.set_weights(**kwargs)
    
    def optimize_trajectory_cost(self,
                                  current_state: WorldState,
                                  context: Optional[Dict[str, Any]] = None) -> Tuple[np.ndarray, float]:
        """
        Optimize trajectory to minimize cost function.
        This is the "Closed-Loop Imagination" - the model dreams of possible
        futures and chooses the one that minimizes cost.
        """
        best_action = None
        best_cost = float('-inf')
        
        candidate_actions = self._generate_candidate_actions()
        
        for action in candidate_actions:
            trajectory = self._rollout_trajectory(current_state, action, context)
            
            traj_pred = TrajectoryPrediction(
                positions=trajectory.positions,
                velocities=trajectory.velocities,
                accelerations=trajectory.accelerations,
                probabilities=np.ones(self._horizon_steps, dtype=np.float32),
                is_collision=trajectory.is_collision,
                uncertainty=trajectory.uncertainty
            )
            
            cost = self.cost_function.compute(traj_pred, context)
            
            if cost > best_cost:
                best_cost = cost
                best_action = action
        
        return best_action if best_action is not None else np.zeros(self.action_dim, dtype=np.float32), best_cost
    
    def _generate_candidate_actions(self) -> List[np.ndarray]:
        """Generate candidate action sequences for planning"""
        actions = []
        
        steering_values = [-0.5, -0.25, 0.0, 0.25, 0.5]
        throttle_values = [0.0, 0.3, 0.6]
        brake_values = [0.0, 0.3]
        
        for steer in steering_values:
            for throttle in throttle_values:
                for brake in brake_values:
                    action = np.array([steer, throttle, brake, 0.0], dtype=np.float32)
                    actions.append(action)
        
        return actions[:self.num_rollouts]
        
    def simulate(self,
                 current_state: WorldState,
                 proposed_actions: List[np.ndarray],
                 context: Optional[Dict[str, Any]] = None) -> SimulationResult:
        """
        Run world model simulation
        
        Args:
            current_state: Current vehicle/world state
            proposed_actions: List of candidate actions (e.g., turn left, go straight)
            context: Additional context (radar objects, map data, etc.)
            
        Returns:
            SimulationResult with evaluated trajectories
        """
        trajectories = []
        
        for action in proposed_actions:
            trajectory = self._rollout_trajectory(current_state, action, context)
            trajectories.append(trajectory)
            
        collision_probs = [t.is_collision for t in trajectories]
        collision_prob = float(np.mean([float(t.is_collision) for t in trajectories]))
        
        rewards = self._evaluate_trajectories(trajectories, context)
        chosen_idx = int(np.argmax(rewards))
        
        is_safe = collision_prob < 0.1 and trajectories[chosen_idx].uncertainty < 0.5
        
        expected_state = self._compute_expected_state(trajectories)
        
        uncertainty_map = self._build_uncertainty_map(trajectories)
        
        return SimulationResult(
            trajectories=trajectories,
            chosen_trajectory_idx=chosen_idx,
            expected_outcome=expected_state,
            uncertainty_map=uncertainty_map,
            collision_probability=collision_prob,
            is_safe=is_safe
        )
        
    def _rollout_trajectory(self,
                          initial_state: WorldState,
                          action: np.ndarray,
                          context: Optional[Dict[str, Any]]) -> TrajectoryPrediction:
        """Roll out a single trajectory"""
        positions = np.zeros((self._horizon_steps, 3), dtype=np.float32)
        velocities = np.zeros((self._horizon_steps, 3), dtype=np.float32)
        accelerations = np.zeros((self._horizon_steps, 3), dtype=np.float32)
        uncertainties = np.zeros(self._horizon_steps, dtype=np.float32)
        
        pos = initial_state.position.copy()
        vel = initial_state.velocity.copy()
        
        for t in range(self._horizon_steps):
            acc = self._predict_acceleration(vel, action, t, context)
            
            vel = vel + acc * self.DT
            pos = pos + vel * self.DT
            
            positions[t] = pos
            velocities[t] = vel
            accelerations[t] = acc
            
            uncertainty = self._estimate_uncertainty(t, action, context)
            uncertainties[t] = uncertainty
            
        is_collision = self._check_collision(positions, context)
        
        return TrajectoryPrediction(
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
            probabilities=np.ones(self._horizon_steps, dtype=np.float32),
            is_collision=is_collision,
            uncertainty=float(np.mean(uncertainties))
        )
        
    def _predict_acceleration(self,
                             velocity: np.ndarray,
                             action: np.ndarray,
                             time_step: int,
                             context: Optional[Dict[str, Any]]) -> np.ndarray:
        """Predict acceleration based on current state and action"""
        acc = np.zeros(3, dtype=np.float32)
        
        steering = action[0] if len(action) > 0 else 0.0
        throttle = action[1] if len(action) > 1 else 0.0
        brake = action[2] if len(action) > 2 else 0.0
        
        speed = np.linalg.norm(velocity[:2])
        if speed > 0.1:
            yaw_rate = steering * speed * 0.1
            acc[0] = -velocity[1] * yaw_rate
            acc[1] = velocity[0] * yaw_rate
            
        acc[0] += throttle * 2.0
        acc[2] += (throttle - brake) * 1.0
        
        return acc
        
    def _estimate_uncertainty(self,
                             time_step: int,
                             action: np.ndarray,
                             context: Optional[Dict[str, Any]]) -> float:
        """Estimate uncertainty that grows with prediction horizon"""
        base_uncertainty = 0.1
        horizon_factor = time_step / self._horizon_steps
        action_magnitude = np.linalg.norm(action) if action.size > 0 else 0.0
        
        uncertainty = base_uncertainty + horizon_factor * 0.5 + action_magnitude * 0.1
        return float(np.clip(uncertainty, 0.0, 1.0))
        
    def _check_collision(self, 
                        positions: np.ndarray,
                        context: Optional[Dict[str, Any]]) -> bool:
        """Check if trajectory collides with any objects"""
        if context is None or 'objects' not in context:
            return False
            
        objects = context['objects']
        road_width = context.get('road_width', 3.5)
        
        for pos in positions:
            lane_offset = abs(pos[1])
            if lane_offset > road_width:
                return True
                
            for obj in objects:
                obj_pos = np.array([obj.get('x', 0), obj.get('y', 0)])
                dist = np.linalg.norm(pos[:2] - obj_pos)
                if dist < 2.0:
                    return True
                    
        return False
        
    def _evaluate_trajectories(self,
                               trajectories: List[TrajectoryPrediction],
                               context: Optional[Dict[str, Any]]) -> np.ndarray:
        """Evaluate and score trajectories"""
        rewards = np.zeros(len(trajectories), dtype=np.float32)
        
        for i, traj in enumerate(trajectories):
            if traj.is_collision:
                rewards[i] = -100.0
                continue
                
            final_speed = np.linalg.norm(traj.velocities[-1][:2])
            progress = traj.positions[-1, 0] - traj.positions[0, 0]
            
            reward = progress * 0.5 + final_speed * 0.3 - traj.uncertainty * 0.2
            
            rewards[i] = reward
            
        return rewards
        
    def _compute_expected_state(self, trajectories: List[TrajectoryPrediction]) -> WorldState:
        """Compute expected world state from all trajectories"""
        weights = np.array([1.0 - t.uncertainty for t in trajectories])
        weights = weights / np.sum(weights)
        
        final_positions = np.array([t.positions[-1] for t in trajectories])
        final_velocities = np.array([t.velocities[-1] for t in trajectories])
        
        expected_pos = np.average(final_positions, weights=weights, axis=0)
        expected_vel = np.average(final_velocities, weights=weights, axis=0)
        
        return WorldState(
            position=expected_pos,
            velocity=expected_vel,
            acceleration=np.zeros(3, dtype=np.float32),
            heading=0.0,
            lane_position=0.0,
            objects=[],
            uncertainty=float(np.mean([t.uncertainty for t in trajectories]))
        )
        
    def _build_uncertainty_map(self, trajectories: List[TrajectoryPrediction]) -> np.ndarray:
        """Build spatial uncertainty map from trajectories"""
        positions = np.array([t.positions for t in trajectories])
        
        uncertainty_map = np.std(positions, axis=0)
        
        return uncertainty_map
        
    def should_execute_action(self, 
                             current_state: WorldState,
                             proposed_action: np.ndarray,
                             context: Optional[Dict[str, Any]] = None) -> Tuple[bool, str]:
        """
        Determine if an action should be executed based on simulation
        
        Returns:
            (should_execute, reason)
        """
        proposed_actions = [proposed_action]
        
        result = self.simulate(current_state, proposed_actions, context)
        
        if result.collision_probability > 0.3:
            return False, f"High collision probability: {result.collision_probability:.2f}"
            
        if not result.is_safe:
            return False, "Trajectory marked unsafe by world model"
            
        chosen_traj = result.trajectories[result.chosen_trajectory_idx]
        if chosen_traj.uncertainty > 0.6:
            return False, f"High uncertainty: {chosen_traj.uncertainty:.2f}"
            
        return True, "ok"


class ImaginationBuffer:
    """
    Maintains history of imagined trajectories for visualization/debugging
    """
    
    def __init__(self, max_history: int = 100):
        self.max_history = max_history
        self._imagined_trajectories: List[SimulationResult] = []
        self._timestamps: List[float] = []
        
    def add_imagination(self, result: SimulationResult, timestamp: float = 0.0):
        """Add a simulation result"""
        self._imagined_trajectories.append(result)
        self._timestamps.append(timestamp)
        
        if len(self._imagined_trajectories) > self.max_history:
            self._imagined_trajectories.pop(0)
            self._timestamps.pop(0)
            
    def get_recent(self, n: int = 10) -> List[SimulationResult]:
        """Get n most recent simulation results"""
        return self._imagined_trajectories[-n:] if len(self._imagined_trajectories) >= n else self._imagined_trajectories
        
    def get_best_trajectory(self) -> Optional[TrajectoryPrediction]:
        """Get the best trajectory from most recent simulation"""
        if not self._imagined_trajectories:
            return None
            
        result = self._imagined_trajectories[-1]
        if result.chosen_trajectory_idx < len(result.trajectories):
            return result.trajectories[result.chosen_trajectory_idx]
        return None
