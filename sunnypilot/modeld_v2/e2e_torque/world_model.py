"""
World Model for Closed-Loop Simulation in sunnypilot
======================================================

This module implements a learned simulator (World Model) where the E2E agent 
can "dream" potential outcomes before taking an action.

Key Features:
- Latent dynamics prediction
- Trajectory rollout simulation
- Uncertainty-aware planning
- Imagination-based safety checks
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict, Any


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


class WorldModel:
    """
    Learned Simulator / World Model
    
    Enables the E2E agent to simulate potential outcomes before executing actions.
    This adds a layer of "imagination" to the current reactive planning.
    
    The model:
    1. Takes current latent state and proposed action
    2. Rolls out multiple possible futures
    3. Evaluates each for safety and desirability
    4. Returns the best trajectory with uncertainty estimates
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
        
    def _build_transition_network(self) -> np.ndarray:
        """Build transition dynamics model (simplified as linear + noise)"""
        return np.random.randn(self.hidden_dim, self.state_dim + self.action_dim).astype(np.float32) * 0.01
        
    def _build_reward_network(self) -> np.ndarray:
        """Build reward/quality prediction model"""
        return np.random.randn(1, self.hidden_dim).astype(np.float32) * 0.01
        
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
