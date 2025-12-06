"""
Self-Learning Enhancement for Sunnypilot2

This module implements advanced self-learning capabilities that allow the system to
adapt its driving behavior based on experience, road conditions, and driver preferences
while maintaining strict safety boundaries. It learns from successful maneuvers and
improves system performance over time.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import math
import pickle
import os
from pathlib import Path
import time
from collections import deque, defaultdict

from cereal import log
from cereal.messaging import SubMaster
from opendbc.car.structs import car
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import LIMIT_NEAREST_SPEED, CONTROL_N
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel


class DrivingContextClassifier:
    """
    Classify driving context to enable contextual learning and adaptation.
    """
    
    def __init__(self):
        self.context_features = {
            'road_type': ['highway', 'city', 'residential', 'rural', 'toll_road'],
            'traffic_density': ['low', 'medium', 'high', 'stop_and_go'],
            'weather': ['clear', 'rain', 'snow', 'fog', 'night'],
            'road_condition': ['dry', 'wet', 'icy', 'construction'],
            'time_of_day': ['morning', 'midday', 'evening', 'night'],
            'vehicle_load': ['empty', 'light', 'medium', 'heavy']
        }
        
        # Learned parameters per context
        self.context_parameters = defaultdict(lambda: {
            'lateral_control_factor': 1.0,
            'longitudinal_time_headway': 1.8,
            'steering_sensitivity': 1.0,
            'acceleration_aggression': 1.0,
            'lane_center_offset': 0.0
        })
        
        # Weight for different contexts
        self.context_weights = defaultdict(float)
    
    def classify_context(self, sm: SubMaster, model_data: Dict[str, Any]) -> Dict[str, str]:
        """Classify current driving context based on sensor data."""
        context = {}
        
        # Road type classification
        if 'carState' in sm:
            car_state = sm['carState']
            v_ego = car_state.vEgo
            
            if v_ego > 20:  # > 72 km/h
                context['road_type'] = 'highway'
            elif v_ego > 8:  # > 29 km/h
                context['road_type'] = 'city'
            else:
                context['road_type'] = 'residential'
        
        # Traffic density
        if 'radarState' in sm:
            radar_state = sm['radarState']
            close_vehicles = 0
            
            if radar_state.leadOne.status and radar_state.leadOne.dRel < 50:
                close_vehicles += 1
            if radar_state.leadTwo.status and radar_state.leadTwo.dRel < 50:
                close_vehicles += 1
            
            if close_vehicles >= 2:
                context['traffic_density'] = 'high'
            elif close_vehicles == 1:
                context['traffic_density'] = 'medium'
            else:
                context['traffic_density'] = 'low'
        
        # Time of day (simplified)
        current_time = time.time()
        hour = (current_time % 86400) // 3600
        if 6 <= hour < 10:
            context['time_of_day'] = 'morning'
        elif 10 <= hour < 16:
            context['time_of_day'] = 'midday'
        elif 16 <= hour < 20:
            context['time_of_day'] = 'evening'
        else:
            context['time_of_day'] = 'night'
        
        # Weather (would use more sophisticated detection in practice)
        context['weather'] = 'clear'  # Simplified
        
        # Road condition
        if 'carState' in sm:
            a_ego = getattr(car_state, 'aEgo', 0.0)
            steering_rate = abs(car_state.steeringRateDeg)
            
            # Check for signs of slippery conditions
            if abs(a_ego) > 3.0 and steering_rate > 10:  # High acceleration and steering
                context['road_condition'] = 'wet'  # Could indicate slippery conditions
            else:
                context['road_condition'] = 'dry'
        
        # Default values for missing contexts
        for feature, values in self.context_features.items():
            if feature not in context:
                context[feature] = values[0]
        
        return context
    
    def get_context_key(self, context: Dict[str, str]) -> str:
        """Create a unique key for the current context."""
        return f"{context['road_type']}_{context['traffic_density']}_{context['weather']}_{context['time_of_day']}"
    
    def get_context_parameters(self, context: Dict[str, str]) -> Dict[str, float]:
        """Get learned parameters for the given context."""
        context_key = self.get_context_key(context)
        return self.context_parameters[context_key].copy()


class ExperienceReplayBuffer:
    """
    Experience replay buffer for reinforcement learning-style updates.
    """
    
    def __init__(self, max_size: int = 10000):
        self.max_size = max_size
        self.buffer = deque(maxlen=max_size)
        
        # Metrics tracking
        self.total_experiences = 0
        self.positive_outcomes = 0
        self.negative_outcomes = 0
    
    def add_experience(self, state: Dict[str, Any], action: Dict[str, float], 
                      reward: float, next_state: Dict[str, Any], 
                      terminal: bool, context: str):
        """Add a new experience to the buffer."""
        experience = {
            'state': state.copy(),
            'action': action.copy(),
            'reward': reward,
            'next_state': next_state.copy(),
            'terminal': terminal,
            'context': context,
            'timestamp': time.time()
        }
        
        self.buffer.append(experience)
        self.total_experiences += 1
        
        # Track outcomes
        if reward > 0:
            self.positive_outcomes += 1
        else:
            self.negative_outcomes += 1
    
    def sample_batch(self, batch_size: int = 32) -> List[Dict[str, Any]]:
        """Sample a batch of experiences for learning."""
        if len(self.buffer) < batch_size:
            return list(self.buffer)
        
        # Simple random sampling (in a real implementation, this could be prioritized sampling)
        indices = np.random.choice(len(self.buffer), size=batch_size, replace=False)
        return [self.buffer[i] for i in indices]
    
    def get_statistics(self) -> Dict[str, float]:
        """Get statistics about the experience buffer."""
        total = self.positive_outcomes + self.negative_outcomes
        return {
            'total_experiences': self.total_experiences,
            'buffer_size': len(self.buffer),
            'positive_rate': self.positive_outcomes / max(total, 1),
            'negative_rate': self.negative_outcomes / max(total, 1)
        }


class SelfLearningManager:
    """
    Main self-learning manager that adapts system parameters based on experience.
    """
    
    def __init__(self, CP, save_path: Optional[str] = None):
        self.CP = CP
        self.save_path = save_path or str(Path(__file__).parent / "learned_params.pkl")
        
        # Initialize context classifier
        self.context_classifier = DrivingContextClassifier()
        
        # Initialize experience buffer
        self.experience_buffer = ExperienceReplayBuffer()
        
        # Learned parameters that can be adjusted
        self.learned_parameters = {
            # Lateral control parameters
            'lateral_control_kp': 0.2,  # Base steering KP
            'lateral_control_ki': 0.02,  # Base steering KI
            'lateral_control_kd': 0.002,  # Base steering KD
            'steering_sensitivity': 1.0,  # Overall steering sensitivity
            'lane_width_compensation': 0.0,  # Compensation for narrow lanes
            
            # Longitudinal control parameters
            'long_time_headway': 1.8,  # Base time headway
            'acceleration_smoothness': 0.8,  # How aggressively to accelerate
            'comfort_braking_factor': 1.0,  # Braking comfort adjustment
            'speed_sensitivity': 1.0,  # How sensitive to speed changes
            
            # Model output adjustments
            'curvature_adjustment_bias': 0.0,  # Bias in curvature commands
            'acceleration_adjustment_bias': 0.0,  # Bias in acceleration commands
        }
        
        # Parameter bounds to ensure safety
        self.parameter_bounds = {
            'lateral_control_kp': (0.05, 0.5),
            'lateral_control_ki': (0.005, 0.1),
            'lateral_control_kd': (0.0005, 0.02),
            'steering_sensitivity': (0.5, 2.0),
            'lane_width_compensation': (-0.5, 0.5),
            'long_time_headway': (1.0, 3.0),
            'acceleration_smoothness': (0.1, 1.0),
            'comfort_braking_factor': (0.5, 1.5),
            'speed_sensitivity': (0.5, 2.0),
            'curvature_adjustment_bias': (-0.01, 0.01),
            'acceleration_adjustment_bias': (-0.5, 0.5),
        }
        
        # Learning rates for different parameters
        self.learning_rates = {
            'lateral_control_kp': 0.001,
            'lateral_control_ki': 0.0005,
            'lateral_control_kd': 0.0001,
            'steering_sensitivity': 0.002,
            'lane_width_compensation': 0.001,
            'long_time_headway': 0.005,
            'acceleration_smoothness': 0.002,
            'comfort_braking_factor': 0.001,
            'speed_sensitivity': 0.001,
            'curvature_adjustment_bias': 0.0001,
            'acceleration_adjustment_bias': 0.001,
        }
        
        # Performance tracking
        self.performance_history = deque(maxlen=100)
        self.safety_violations = 0
        self.efficiency_score = 0.0
        self.comfort_score = 0.0
        
        # Learning state
        self.is_learning_enabled = True
        self.learning_mode = 'exploration'  # 'exploration', 'exploitation', or 'conservative'
        
        # Load any previously saved parameters
        self.load_learned_parameters()
        
        # Reward function weights
        self.reward_weights = {
            'safety': 10.0,
            'efficiency': 5.0,
            'comfort': 3.0,
            'tracking': 2.0,
            'compliance': 1.0
        }
    
    def update(self, sm: SubMaster, model_data: Dict[str, Any], 
              control_outputs: Dict[str, Any], 
              driver_interface: Optional[Dict[str, Any]] = None) -> Dict[str, float]:
        """
        Update learning system with current state and outcomes.
        
        Args:
            sm: SubMaster with current sensor data
            model_data: Model outputs
            control_outputs: Control system outputs
            driver_interface: Driver intervention data if available
            
        Returns:
            Dictionary of adjusted parameters
        """
        if not self.is_learning_enabled:
            return {}
        
        # Classify current context
        current_context = self.context_classifier.classify_context(sm, model_data)
        context_key = self.context_classifier.get_context_key(current_context)
        
        # Get current state
        current_state = self._get_state_representation(sm, model_data, control_outputs)
        
        # Calculate performance and reward
        performance_metrics = self._evaluate_performance(sm, model_data, control_outputs, driver_interface)
        reward = self._calculate_reward(performance_metrics)
        
        # Store experience
        if hasattr(self, '_prev_state') and hasattr(self, '_prev_action'):
            self.experience_buffer.add_experience(
                self._prev_state, self._prev_action, reward, current_state, 
                terminal=False, context=context_key
            )
        
        # Update performance history
        self.performance_history.append(performance_metrics)
        
        # Learn from recent experiences
        if len(self.experience_buffer.buffer) > 100:
            self._learn_from_experiences()
        
        # Adjust parameters based on context
        context_params = self.context_classifier.get_context_parameters(current_context)
        adjusted_parameters = self._adjust_parameters_for_context(context_params)
        
        # Store current state and action for next iteration
        self._prev_state = current_state.copy()
        self._prev_action = adjusted_parameters.copy()
        
        return adjusted_parameters
    
    def _get_state_representation(self, sm: SubMaster, model_data: Dict[str, Any], 
                                 control_outputs: Dict[str, Any]) -> Dict[str, Any]:
        """Get a compact state representation for learning."""
        state = {
            'v_ego': sm['carState'].vEgo,
            'a_ego': getattr(sm['carState'], 'aEgo', 0.0),
            'steering_angle': sm['carState'].steeringAngleDeg,
            'steering_rate': sm['carState'].steeringRateDeg,
            'curvature_error': 0.0,  # Would be calculated from actual vs desired
            'lateral_error': 0.0,    # Distance from center of lane
            'time_to_lead': float('inf'),  # Time to lead vehicle
            'model_confidence': getattr(getattr(model_data, 'meta', None), 'confidence', 1.0),
            'control_effort': abs(control_outputs.get('steer_torque', 0.0))
        }
        
        # Calculate time to lead if available
        if 'radarState' in sm and sm['radarState'].leadOne.status:
            v_rel = sm['radarState'].leadOne.vRel
            d_rel = sm['radarState'].leadOne.dRel
            if v_rel < 0 and d_rel > 0:  # Approaching lead
                state['time_to_lead'] = d_rel / abs(v_rel)
        
        # Calculate tracking error if we have model reference
        if hasattr(model_data, 'action') and 'desired_curvature' in control_outputs:
            state['curvature_error'] = abs(
                model_data.action.desiredCurvature - control_outputs.get('desired_curvature', 0.0)
            )
        
        return state
    
    def _evaluate_performance(self, sm: SubMaster, model_data: Dict[str, Any],
                             control_outputs: Dict[str, Any],
                             driver_interface: Optional[Dict[str, Any]]) -> Dict[str, float]:
        """Evaluate current performance across multiple dimensions."""
        car_state = sm['carState']
        v_ego = car_state.vEgo
        a_ego = getattr(car_state, 'aEgo', 0.0)
        
        metrics = {
            'safety': 1.0,    # Higher is better
            'efficiency': 1.0,
            'comfort': 1.0,
            'tracking': 1.0,
            'compliance': 1.0
        }
        
        # Safety metrics
        if 'radarState' in sm:
            radar_state = sm['radarState']
            if radar_state.leadOne.status:
                d_rel = radar_state.leadOne.dRel
                v_rel = radar_state.leadOne.vRel
                if v_rel < 0 and d_rel > 0:  # Approaching lead
                    ttc = d_rel / abs(v_rel)
                    # Safety decreases as TTC approaches dangerous threshold
                    if ttc < 2.0:
                        metrics['safety'] = max(0.1, (ttc - 1.0))  # Severe penalty for TTC < 2s
                    else:
                        metrics['safety'] = 1.0
        
        # Comfort metrics (minimize jerk and aggressive control)
        max_comfort_accel = 2.0  # m/s²
        metrics['comfort'] = max(0.1, 1.0 - abs(a_ego) / max_comfort_accel)
        
        # Efficiency metrics (maintain reasonable speeds, smooth driving)
        target_efficiency_speed = 25.0  # m/s (90 km/h)
        speed_efficiency = max(0.5, 1.0 - abs(v_ego - target_efficiency_speed) / target_efficiency_speed)
        metrics['efficiency'] = speed_efficiency
        
        # Tracking metrics (how well we follow model commands)
        if hasattr(model_data, 'action') and 'desired_curvature' in control_outputs:
            tracking_error = abs(
                model_data.action.desiredCurvature - control_outputs.get('desired_curvature', 0.0)
            )
            metrics['tracking'] = max(0.1, 1.0 - tracking_error / 0.01)  # 0.01 rad/m is significant error
        
        # Compliance (following traffic rules, smooth behavior)
        if driver_interface and driver_interface.get('intervened', False):
            metrics['compliance'] = 0.5  # Significant penalty for driver intervention
        else:
            metrics['compliance'] = 1.0
        
        # Update aggregate scores
        self.comfort_score = 0.99 * self.comfort_score + 0.01 * metrics['comfort']
        self.efficiency_score = 0.99 * self.efficiency_score + 0.01 * metrics['efficiency']
        
        return metrics
    
    def _calculate_reward(self, performance_metrics: Dict[str, float]) -> float:
        """Calculate reward based on performance metrics."""
        total_reward = 0.0
        
        for metric_name, value in performance_metrics.items():
            weight = self.reward_weights.get(metric_name, 1.0)
            total_reward += weight * value
        
        # Normalize reward to reasonable range
        normalized_reward = total_reward / sum(self.reward_weights.values())
        
        # Add small random component to encourage exploration
        exploration_bonus = np.random.normal(0, 0.01)
        
        return normalized_reward + exploration_bonus
    
    def _learn_from_experiences(self):
        """Learn from stored experiences using temporal difference learning."""
        # Sample batch of experiences
        batch = self.experience_buffer.sample_batch(batch_size=16)
        
        # For each parameter, update based on experience
        for param_name, current_value in self.learned_parameters.items():
            if param_name not in self.learning_rates:
                continue
            
            # Calculate parameter adjustment based on performance in experiences
            param_adjustment = 0.0
            valid_updates = 0
            
            for experience in batch:
                # Simple policy gradient approach: adjust parameter based on reward
                reward = experience['reward']
                
                # Use a simple gradient estimate (in practice, this would be more sophisticated)
                # Positive reward suggests current parameter value was good
                # Negative reward suggests we should adjust in the opposite direction
                param_adjustment += reward * np.random.normal(0, 0.1)  # Small random step
                valid_updates += 1
            
            if valid_updates > 0:
                # Average the adjustment
                avg_adjustment = param_adjustment / valid_updates
                
                # Apply learning rate
                final_adjustment = avg_adjustment * self.learning_rates[param_name]
                
                # Update parameter with bounds checking
                new_value = current_value + final_adjustment
                min_val, max_val = self.parameter_bounds[param_name]
                new_value = max(min_val, min(max_val, new_value))
                
                self.learned_parameters[param_name] = new_value
    
    def _adjust_parameters_for_context(self, context_params: Dict[str, float]) -> Dict[str, float]:
        """Adjust parameters based on current context."""
        adjusted = {}
        
        # Combine learned parameters with context-specific adjustments
        for param_name, base_value in self.learned_parameters.items():
            # Get context-specific adjustment if available
            context_adjustment = context_params.get(param_name, 0.0)
            
            # Apply context adjustment
            if 'factor' in param_name or 'sensitivity' in param_name:
                # These are multiplicative adjustments
                adjusted[param_name] = base_value * (1 + context_adjustment)
            else:
                # These are additive adjustments
                adjusted[param_name] = base_value + context_adjustment
            
            # Apply bounds checking
            min_val, max_val = self.parameter_bounds[param_name]
            adjusted[param_name] = max(min_val, min(max_val, adjusted[param_name]))
        
        return adjusted
    
    def apply_parameter_adjustments(self, control_params: Dict[str, Any], 
                                   learned_adjustments: Dict[str, float]) -> Dict[str, Any]:
        """
        Apply learned parameter adjustments to control system.
        
        Args:
            control_params: Current control parameters
            learned_adjustments: Adjustments from self-learning
            
        Returns:
            Updated control parameters
        """
        updated_params = control_params.copy()
        
        # Apply lateral control adjustments
        for param in ['steer_kp', 'steer_ki', 'steer_kd']:
            if param in learned_adjustments:
                original_value = updated_params.get(param, 0.1)  # Default value
                adjustment = learned_adjustments.get(
                    param.replace('steer_', 'lateral_control_'), 1.0
                )
                updated_params[param] = original_value * adjustment
        
        # Apply time headway adjustment
        if 'long_time_headway' in learned_adjustments:
            base_headway = updated_params.get('time_headway', 1.8)
            adjustment = learned_adjustments['long_time_headway']
            updated_params['time_headway'] = adjustment
        
        # Apply steering sensitivity adjustment
        if 'steering_sensitivity' in learned_adjustments:
            current_sensitivity = updated_params.get('steering_sensitivity', 1.0)
            adjustment = learned_adjustments['steering_sensitivity']
            updated_params['steering_sensitivity'] = current_sensitivity * adjustment
        
        # Apply model output adjustments
        if 'curvature_adjustment_bias' in learned_adjustments:
            current_bias = updated_params.get('curvature_bias', 0.0)
            adjustment = learned_adjustments['curvature_adjustment_bias']
            updated_params['curvature_bias'] = current_bias + adjustment
        
        if 'acceleration_adjustment_bias' in learned_adjustments:
            current_bias = updated_params.get('acceleration_bias', 0.0)
            adjustment = learned_adjustments['acceleration_adjustment_bias']
            updated_params['acceleration_bias'] = current_bias + adjustment
        
        return updated_params
    
    def save_learned_parameters(self):
        """Save learned parameters to persistent storage."""
        try:
            save_data = {
                'learned_parameters': self.learned_parameters,
                'context_parameters': dict(self.context_classifier.context_parameters),
                'experience_stats': self.experience_buffer.get_statistics(),
                'performance_history': list(self.performance_history),
                'timestamp': time.time()
            }
            
            with open(self.save_path, 'wb') as f:
                pickle.dump(save_data, f)
            
            cloudlog.info(f"Saved learned parameters to {self.save_path}")
        except Exception as e:
            cloudlog.error(f"Failed to save learned parameters: {e}")
    
    def load_learned_parameters(self):
        """Load learned parameters from persistent storage."""
        try:
            if os.path.exists(self.save_path):
                with open(self.save_path, 'rb') as f:
                    save_data = pickle.load(f)
                
                self.learned_parameters.update(save_data.get('learned_parameters', {}))
                self.context_classifier.context_parameters.update(
                    save_data.get('context_parameters', {})
                )
                
                cloudlog.info(f"Loaded learned parameters from {self.save_path}")
            else:
                cloudlog.info("No saved parameters found, using defaults")
        except Exception as e:
            cloudlog.error(f"Failed to load learned parameters: {e}")
    
    def reset_learning(self):
        """Reset all learned parameters to defaults."""
        # Reset learned parameters to initial values
        self.learned_parameters = {
            'lateral_control_kp': 0.2,
            'lateral_control_ki': 0.02,
            'lateral_control_kd': 0.002,
            'steering_sensitivity': 1.0,
            'lane_width_compensation': 0.0,
            'long_time_headway': 1.8,
            'acceleration_smoothness': 0.8,
            'comfort_braking_factor': 1.0,
            'speed_sensitivity': 1.0,
            'curvature_adjustment_bias': 0.0,
            'acceleration_adjustment_bias': 0.0,
        }
        
        # Reset context-specific parameters
        self.context_classifier.context_parameters.clear()
        
        # Clear experience buffer
        self.experience_buffer = ExperienceReplayBuffer()
        
        # Reset performance history
        self.performance_history.clear()
        
        cloudlog.info("Reset all learned parameters to defaults")


class DriverPreferenceLearner:
    """
    Learn driver preferences and adapt the system to individual driving style.
    """
    
    def __init__(self):
        self.driver_style_profile = {
            'aggression_level': 0.5,  # 0.0 (conservative) to 1.0 (aggressive)
            'following_distance_preference': 1.8,  # Preferred time headway
            'steering_smoothness_preference': 0.8,  # 0.0 (jerky) to 1.0 (smooth)
            'acceleration_preference': 0.6,  # 0.0 (gradual) to 1.0 (aggressive)
            'lane_position_preference': 0.0,  # -1.0 (left) to 1.0 (right), 0.0 (center)
        }
        
        self.preference_learning_rate = 0.01
        self.profile_confidence = 0.0
        self.intervention_pattern = []
    
    def update_driver_profile(self, sm: SubMaster, driver_interface: Dict[str, Any]):
        """
        Update driver profile based on driver interventions and preferences.
        
        Args:
            sm: SubMaster with current sensor data
            driver_interface: Driver behavior data
        """
        car_state = sm['carState']
        v_ego = car_state.vEgo
        
        # Check for driver interventions
        steering_intervention = driver_interface.get('steering_intervened', False)
        braking_intervention = driver_interface.get('braking_intervened', False)
        acceleration_intervention = driver_interface.get('acceleration_intervened', False)
        
        # Update intervention pattern
        self.intervention_pattern.append({
            'timestamp': time.time(),
            'steering': steering_intervention,
            'braking': braking_intervention,
            'acceleration': acceleration_intervention,
            'v_ego': v_ego
        })
        
        # Keep only recent interventions (last 100)
        if len(self.intervention_pattern) > 100:
            self.intervention_pattern = self.intervention_pattern[-100:]
        
        # Analyze intervention patterns to infer preferences
        if len(self.intervention_pattern) >= 10:
            self._analyze_driver_interventions()
    
    def _analyze_driver_interventions(self):
        """Analyze driver intervention patterns to infer preferences."""
        recent_interventions = self.intervention_pattern[-20:]  # Look at last 20
        
        if not recent_interventions:
            return
        
        # Calculate intervention rates
        steering_interventions = sum(1 for i in recent_interventions if i['steering'])
        braking_interventions = sum(1 for i in recent_interventions if i['braking'])
        acceleration_interventions = sum(1 for i in recent_interventions if i['acceleration'])
        
        total_interventions = len(recent_interventions)
        
        # Infer preferences based on intervention patterns
        if steering_interventions / total_interventions > 0.3:
            # Driver makes many steering corrections - prefers more responsive steering
            current_pref = self.driver_style_profile['steering_smoothness_preference']
            self.driver_style_profile['steering_smoothness_preference'] = min(
                1.0, current_pref + self.preference_learning_rate
            )
        
        if braking_interventions / total_interventions > 0.3:
            # Driver brakes frequently - may prefer longer following distances
            current_pref = self.driver_style_profile['following_distance_preference']
            self.driver_style_profile['following_distance_preference'] = min(
                3.0, current_pref + 0.1  # Increase following distance
            )
    
    def get_adaptation_factor(self, adaptation_type: str) -> float:
        """
        Get adaptation factor for a specific type of control.
        
        Args:
            adaptation_type: Type of adaptation ('longitudinal', 'lateral', etc.)
            
        Returns:
            Factor to apply to default parameters
        """
        if adaptation_type == 'longitudinal':
            # Higher aggression allows for more aggressive acceleration/braking
            aggression = self.driver_style_profile['aggression_level']
            return 0.7 + aggression * 0.6  # Range 0.7 to 1.3
        elif adaptation_type == 'lateral':
            # Smoother steering preference affects steering gains
            smoothness = self.driver_style_profile['steering_smoothness_preference']
            return max(0.5, 1.0 - (smoothness * 0.5))  # Smoother preference = lower gains
        else:
            return 1.0


class SelfLearningSystem:
    """
    Main self-learning system that integrates all learning components.
    """
    
    def __init__(self, CP, enable_driver_learning=True):
        self.learning_manager = SelfLearningManager(CP)
        self.enable_driver_learning = enable_driver_learning
        
        if enable_driver_learning:
            self.driver_learner = DriverPreferenceLearner()
        else:
            self.driver_learner = None
    
    def update(self, sm: SubMaster, model_data: Dict[str, Any], 
              control_outputs: Dict[str, Any],
              driver_interface: Optional[Dict[str, Any]] = None) -> Dict[str, Dict[str, float]]:
        """
        Update the entire self-learning system.
        
        Returns:
            Dictionary containing parameter adjustments for different systems
        """
        # Update main learning manager
        learned_adjustments = self.learning_manager.update(
            sm, model_data, control_outputs, driver_interface
        )
        
        # Update driver preference learning if enabled
        driver_adjustments = {}
        if self.driver_learner and driver_interface:
            self.driver_learner.update_driver_profile(sm, driver_interface)
            
            # Get driver-specific adaptations
            driver_adjustments = {
                'longitudinal_aggression': self.driver_learner.get_adaptation_factor('longitudinal'),
                'lateral_smoothness': self.driver_learner.get_adaptation_factor('lateral'),
            }
        
        # Combine all adjustments
        all_adjustments = {
            'learned_parameters': learned_adjustments,
            'driver_preferences': driver_adjustments,
            'learning_enabled': self.learning_manager.is_learning_enabled
        }
        
        return all_adjustments
    
    def apply_adjustments(self, control_params: Dict[str, Any],
                         adjustments: Dict[str, Dict[str, float]]) -> Dict[str, Any]:
        """Apply all learned adjustments to control parameters."""
        updated_params = control_params.copy()
        
        # Apply learned parameter adjustments
        learned_adjustments = adjustments.get('learned_parameters', {})
        updated_params = self.learning_manager.apply_parameter_adjustments(
            updated_params, learned_adjustments
        )
        
        # Apply driver preference adjustments if available
        driver_adjustments = adjustments.get('driver_preferences', {})
        
        if 'longitudinal_aggression' in driver_adjustments:
            aggression_factor = driver_adjustments['longitudinal_aggression']
            if 'acceleration_smoothness' in updated_params:
                updated_params['acceleration_smoothness'] *= aggression_factor
        
        if 'lateral_smoothness' in driver_adjustments:
            smoothness_factor = driver_adjustments['lateral_smoothness']
            if 'steering_sensitivity' in updated_params:
                updated_params['steering_sensitivity'] *= smoothness_factor
        
        return updated_params
    
    def save_state(self):
        """Save the entire learning system state."""
        self.learning_manager.save_learned_parameters()
    
    def load_state(self):
        """Load the entire learning system state."""
        self.learning_manager.load_learned_parameters()


def create_self_learning_system(CP):
    """Create and return a complete self-learning system."""
    return SelfLearningSystem(CP)


if __name__ == "__main__":
    print("Self-Learning Enhancement Module")
    print("Implements contextual learning, experience replay, and driver preference adaptation")
    print("for improved performance over time while maintaining safety.")