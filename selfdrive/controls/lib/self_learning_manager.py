#!/usr/bin/env python3
"""
Self-Learning Manager for Adaptive Autonomous Driving

This module implements online learning capabilities for sunnypilot that adapt
the driving behavior based on real-time feedback and driver interventions.
The system learns from driver corrections and adjusts model parameters accordingly.
"""

import numpy as np
import time
from collections import deque
from typing import Dict, List, Tuple, Optional
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from cereal import log


class SelfLearningManager:
    """
    Manages self-learning capabilities for autonomous driving.
    
    The system learns from driver interventions, model-actual discrepancies,
    and environmental conditions to adapt driving parameters and model outputs.
    """
    
    def __init__(self, CP, CP_SP):
        self.CP = CP
        self.CP_SP = CP_SP
        self.params = Params()
        
        # Initialize learning state
        self.learning_enabled = True
        self.learning_rate = 0.01
        self.confidence_threshold = 0.7
        self.intervention_threshold = 0.5  # Amount of steering correction to trigger learning
        
        # Memory for storing recent experiences
        self.experience_buffer = deque(maxlen=1000)  # Store recent driving experiences
        self.intervention_buffer = deque(maxlen=100)  # Store interventions
        self.model_accuracy_history = deque(maxlen=500)  # Track model prediction accuracy
        
        # Adaptive parameters that will be learned
        self.adaptive_params = {
            'lateral_control_factor': 1.0,  # Scaling factor for lateral control
            'curvature_bias': 0.0,  # Bias adjustment to desired curvature
            'acceleration_factor': 1.0,  # Scaling factor for longitudinal acceleration
            'reaction_time_compensation': 0.2  # Time compensation in seconds
        }
        
        # Learning state tracking
        self.last_update_time = time.time()
        self.update_interval = 5.0  # Update parameters every 5 seconds
        self.learning_samples = 0
        self.total_samples = 0
        
        # Initialize from saved parameters if available
        self.load_adaptive_params()
        
        cloudlog.info("Self-Learning Manager initialized")
    
    def update_from_driver_intervention(self, CS, desired_curvature: float, actual_curvature: float, 
                                      steering_torque: float, v_ego: float):
        """
        Update learning system based on driver intervention.
        
        Args:
            CS: CarState message
            desired_curvature: Model's desired curvature
            actual_curvature: Actual measured curvature from vehicle
            steering_torque: Current steering torque
            v_ego: Vehicle speed
        """
        if not self.learning_enabled:
            return
            
        # Detect if driver is overriding the system
        steering_pressed = CS.steeringPressed
        if not steering_pressed:
            return
            
        # Calculate the difference between desired and actual
        curvature_error = desired_curvature - actual_curvature
        torque_magnitude = abs(steering_torque)
        
        # Only learn if the correction is significant
        if torque_magnitude > self.intervention_threshold:
            experience = {
                'timestamp': time.time(),
                'desired_curvature': desired_curvature,
                'actual_curvature': actual_curvature,
                'curvature_error': curvature_error,
                'steering_torque': steering_torque,
                'v_ego': v_ego,
                'road_type': self._classify_road_type(v_ego, abs(desired_curvature)),
                'intervention_type': 'steering_override'
            }
            
            self.intervention_buffer.append(experience)
            self.experience_buffer.append(experience)
            
            # Adjust parameters based on intervention
            self._adapt_from_intervention(experience)
    
    def update_from_model_accuracy(self, desired_curvature: float, actual_curvature: float, 
                                 v_ego: float, model_confidence: float = 1.0):
        """
        Update learning system based on model prediction accuracy.
        
        Args:
            desired_curvature: Model's desired curvature
            actual_curvature: Actual measured curvature from vehicle
            v_ego: Vehicle speed
            model_confidence: Confidence score from the model (0.0-1.0)
        """
        if not self.learning_enabled:
            return
            
        # Calculate prediction error
        prediction_error = abs(desired_curvature - actual_curvature)
        
        # Track model accuracy over time
        accuracy_record = {
            'timestamp': time.time(),
            'error': prediction_error,
            'confidence': model_confidence,
            'v_ego': v_ego,
            'adjustment_needed': prediction_error > 0.01  # Significant error threshold
        }
        
        self.model_accuracy_history.append(accuracy_record)
        
        # Store experience if there's a meaningful discrepancy
        if accuracy_record['adjustment_needed'] and model_confidence > self.confidence_threshold:
            experience = {
                'timestamp': time.time(),
                'type': 'model_accuracy',
                'desired_curvature': desired_curvature,
                'actual_curvature': actual_curvature,
                'prediction_error': prediction_error,
                'model_confidence': model_confidence,
                'v_ego': v_ego
            }
            self.experience_buffer.append(experience)
    
    def adjust_curvature_prediction(self, original_curvature: float, v_ego: float) -> float:
        """
        Apply learned adjustments to the desired curvature prediction.
        
        Args:
            original_curvature: Original model output curvature
            v_ego: Vehicle speed
            
        Returns:
            Adjusted curvature value based on learned parameters
        """
        if not self.learning_enabled:
            return original_curvature
            
        # Apply learned adjustments
        adjusted_curvature = original_curvature * self.adaptive_params['lateral_control_factor']
        adjusted_curvature += self.adaptive_params['curvature_bias']
        
        # Apply speed-dependent adjustments
        speed_factor = max(0.8, min(1.2, 1.0 + (v_ego - 15.0) * 0.005))  # Adjust for speed
        adjusted_curvature *= speed_factor
        
        return adjusted_curvature
    
    def adjust_acceleration_prediction(self, original_accel: float, v_ego: float) -> float:
        """
        Apply learned adjustments to the desired acceleration prediction.
        
        Args:
            original_accel: Original model output acceleration
            v_ego: Vehicle speed
            
        Returns:
            Adjusted acceleration value based on learned parameters
        """
        if not self.learning_enabled:
            return original_accel
            
        # Apply learned acceleration factor
        adjusted_accel = original_accel * self.adaptive_params['acceleration_factor']
        
        return adjusted_accel
    
    def _adapt_from_intervention(self, experience: Dict):
        """
        Adapt system parameters based on a driver intervention experience.
        
        Args:
            experience: Experience record containing intervention details
        """
        curvature_error = experience['curvature_error']
        v_ego = experience['v_ego']
        
        # Adjust lateral control factor based on systematic errors
        # If desired is consistently different from actual, adjust the scaling
        if abs(curvature_error) > 0.01:  # Significant error
            # Update lateral control factor slowly
            current_factor = self.adaptive_params['lateral_control_factor']
            adjustment = -curvature_error * self.learning_rate * (1.0 + v_ego * 0.01)  # More adjustment at higher speeds
            new_factor = current_factor + adjustment
            
            # Constrain factor to reasonable range
            new_factor = max(0.7, min(1.3, new_factor))
            self.adaptive_params['lateral_control_factor'] = new_factor
            
        # Adjust curvature bias if there's a systematic offset
        if abs(curvature_error) > 0.005:
            current_bias = self.adaptive_params['curvature_bias']
            bias_adjustment = -curvature_error * self.learning_rate * 0.5  # Slower bias adjustment
            new_bias = current_bias + bias_adjustment
            self.adaptive_params['curvature_bias'] = new_bias
        
        self.learning_samples += 1
        self._save_adaptive_params_if_needed()
    
    def _classify_road_type(self, v_ego: float, curvature: float) -> str:
        """
        Classify the current road type based on speed and curvature.
        
        Args:
            v_ego: Vehicle speed
            curvature: Road curvature
            
        Returns:
            Road type classification
        """
        if v_ego < 5.0:  # ~18 km/h
            return 'low_speed_urban'
        elif v_ego < 15.0:  # ~54 km/h
            return 'city_roads'
        elif v_ego < 25.0:  # ~90 km/h
            return 'highway_entry'
        else:
            return 'highway'
    
    def periodic_update(self):
        """
        Perform periodic learning updates based on accumulated data.
        This should be called periodically to update parameters based on recent experiences.
        """
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return
            
        # Update learning based on recent experiences
        if len(self.model_accuracy_history) > 10:
            # Calculate average prediction error
            recent_errors = [rec['error'] for rec in list(self.model_accuracy_history)[-20:]]
            avg_error = np.mean(recent_errors) if recent_errors else 0
            
            # Adjust parameters based on overall accuracy
            if avg_error > 0.02:  # High error threshold
                # Increase learning rate temporarily for faster adaptation
                self.learning_rate = min(0.02, self.learning_rate * 1.1)
            else:
                # Decrease learning rate to stabilize
                self.learning_rate = max(0.005, self.learning_rate * 0.95)
        
        # Regularize parameters to prevent excessive drift
        self._regularize_parameters()
        
        self.last_update_time = current_time
        
        # Log learning statistics periodically
        if self.learning_samples % 50 == 0:
            cloudlog.info(f"Self-Learning Stats - Factor: {self.adaptive_params['lateral_control_factor']:.3f}, "
                         f"Bias: {self.adaptive_params['curvature_bias']:.5f}, "
                         f"Samples: {self.learning_samples}")
    
    def _regularize_parameters(self):
        """
        Apply regularization to prevent parameters from drifting too far from baseline.
        """
        # Regularize lateral control factor towards 1.0
        factor = self.adaptive_params['lateral_control_factor']
        if factor > 1.2 or factor < 0.8:
            # Gradually pull back to center
            target = 1.0
            self.adaptive_params['lateral_control_factor'] = 0.99 * factor + 0.01 * target
        
        # Regularize curvature bias towards 0.0
        bias = self.adaptive_params['curvature_bias']
        if abs(bias) > 0.01:
            # Apply regularization with exponentially decaying weight
            self.adaptive_params['curvature_bias'] *= 0.995
    
    def save_learning_state(self):
        """
        Save the current learning state to persistent storage.
        """
        try:
            learning_state = {
                'adaptive_params': self.adaptive_params,
                'learning_rate': self.learning_rate,
                'learning_samples': self.learning_samples,
                'timestamp': time.time()
            }
            self.params.put("SelfLearningState", str(learning_state))
            cloudlog.info("Learning state saved")
        except Exception as e:
            cloudlog.error(f"Failed to save learning state: {e}")
    
    def load_adaptive_params(self):
        """
        Load adaptive parameters from persistent storage if available.
        """
        try:
            saved_state_str = self.params.get("SelfLearningState", encoding='utf-8')
            if saved_state_str:
                import ast
                saved_state = ast.literal_eval(saved_state_str)
                
                # Load adaptive parameters if they exist
                if 'adaptive_params' in saved_state:
                    for key, value in saved_state['adaptive_params'].items():
                        if key in self.adaptive_params:
                            self.adaptive_params[key] = value
                
                # Load other parameters
                if 'learning_rate' in saved_state:
                    self.learning_rate = saved_state['learning_rate']
                if 'learning_samples' in saved_state:
                    self.learning_samples = saved_state['learning_samples']
                    
                cloudlog.info(f"Learning state loaded - samples: {self.learning_samples}")
        except Exception as e:
            cloudlog.warning(f"Failed to load learning state: {e}")
    
    def _save_adaptive_params_if_needed(self):
        """
        Save adaptive parameters periodically to persistent storage.
        """
        if self.learning_samples % 100 == 0:  # Save every 100 learning samples
            self.save_learning_state()
    
    def reset_learning_state(self):
        """
        Reset the learning state to initial values.
        """
        self.adaptive_params = {
            'lateral_control_factor': 1.0,
            'curvature_bias': 0.0,
            'acceleration_factor': 1.0,
            'reaction_time_compensation': 0.2
        }
        self.learning_samples = 0
        self.experience_buffer.clear()
        self.intervention_buffer.clear()
        self.model_accuracy_history.clear()
        self.save_learning_state()
        cloudlog.info("Learning state reset to initial values")


class ModelEnhancer:
    """
    Enhances model outputs with learned adjustments.
    """
    
    def __init__(self, self_learning_manager: SelfLearningManager):
        self.manager = self_learning_manager
    
    def enhance_curvature_prediction(self, base_curvature: float, v_ego: float) -> float:
        """
        Enhance curvature prediction using learned parameters.
        
        Args:
            base_curvature: Base curvature from the neural network model
            v_ego: Vehicle speed
            
        Returns:
            Enhanced curvature prediction
        """
        return self.manager.adjust_curvature_prediction(base_curvature, v_ego)
    
    def enhance_acceleration_prediction(self, base_accel: float, v_ego: float) -> float:
        """
        Enhance acceleration prediction using learned parameters.
        
        Args:
            base_accel: Base acceleration from the neural network model
            v_ego: Vehicle speed
            
        Returns:
            Enhanced acceleration prediction
        """
        return self.manager.adjust_acceleration_prediction(base_accel, v_ego)