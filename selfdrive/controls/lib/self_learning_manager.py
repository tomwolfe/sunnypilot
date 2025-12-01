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

        # Track performance metrics
        self.performance_start_time = time.time()
        self.total_updates = 0
        self.max_update_time = 0.0
        self.update_time_samples = deque(maxlen=100)  # Track recent update times

        # Memory for storing recent experiences
        self.experience_buffer = deque(maxlen=1000)  # Store recent driving experiences
        self.intervention_buffer = deque(maxlen=100)  # Store interventions
        self.model_accuracy_history = deque(maxlen=500)  # Track model prediction accuracy

        # Adaptive parameters that will be learned
        self.adaptive_params = {
            'lateral_control_factor': 1.0,  # Scaling factor for lateral control
            'curvature_bias': 0.0,  # Bias adjustment to desired curvature
            'acceleration_factor': 1.0,  # Scaling factor for longitudinal acceleration
            'reaction_time_compensation': 0.2,  # Time compensation in seconds
            'speed_compensation_base': 1.0,  # Base value for speed compensation
            'speed_compensation_rate': 0.0  # Rate of change with speed (learnable instead of hardcoded)
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
                                      steering_torque: float, v_ego: float, model_prediction_error: float = None, model_confidence: float = 1.0):
        """
        Update learning system based on driver intervention.

        Args:
            CS: CarState message
            desired_curvature: Model's desired curvature
            actual_curvature: Actual measured curvature from vehicle
            steering_torque: Current steering torque
            v_ego: Vehicle speed
            model_prediction_error: Difference between model output and actual vehicle behavior
            model_confidence: Confidence score from the model (0.0-1.0)
        """
        if not self.learning_enabled:
            return

        # Performance tracking
        start_time = time.time()

        # Detect if driver is overriding the system
        steering_pressed = CS.steeringPressed
        if not steering_pressed:
            # Track the update for performance monitoring
            update_time = time.time() - start_time
            self._update_performance_metrics(update_time)
            return

        # Validate model confidence to ensure it's within expected range
        if model_confidence < 0.0 or model_confidence > 1.0:
            cloudlog.warning(f"Invalid model confidence value: {model_confidence}, clamping to valid range")
            model_confidence = np.clip(model_confidence, 0.0, 1.0)

        # Calculate the difference between desired and actual
        curvature_error = desired_curvature - actual_curvature
        torque_magnitude = abs(steering_torque)

        # Context-aware learning: only learn when it's likely a corrective action
        # Check if there's a significant model prediction error and driver is correcting it
        model_error_significant = model_prediction_error is not None and abs(model_prediction_error) > 0.02
        correction_direction_matches = (model_prediction_error is not None and
                                       np.sign(model_prediction_error) == np.sign(steering_torque))

        # Additional checks for context-aware learning
        high_model_error = model_error_significant and abs(model_prediction_error) > 0.05
        low_model_confidence = model_confidence < self.confidence_threshold  # Use actual model confidence instead of hardcoded False
        driver_correction = torque_magnitude > self.intervention_threshold

        # Only learn if it's likely a corrective action (high model error or correction in the right direction) and model confidence is sufficient
        should_learn = (high_model_error or (model_error_significant and correction_direction_matches)) and driver_correction and not low_model_confidence

        # Additional safety check - don't learn if the model is performing well
        if model_error_significant and abs(curvature_error) < 0.01 and torque_magnitude < 0.5:
            should_learn = False

        if should_learn:
            experience = {
                'timestamp': time.time(),
                'desired_curvature': desired_curvature,
                'actual_curvature': actual_curvature,
                'curvature_error': curvature_error,
                'steering_torque': steering_torque,
                'v_ego': v_ego,
                'model_prediction_error': model_prediction_error,
                'model_confidence': model_confidence,
                'road_type': self._classify_road_type(v_ego, abs(desired_curvature)),
                'intervention_type': 'corrective_steering'
            }

            self.intervention_buffer.append(experience)
            self.experience_buffer.append(experience)

            # Adjust parameters based on intervention
            self._adapt_from_intervention(experience)

        # Track the update for performance monitoring
        update_time = time.time() - start_time
        self._update_performance_metrics(update_time)
    
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

        # Performance tracking
        start_time = time.time()

        # Validate model confidence to ensure it's within expected range
        if model_confidence < 0.0 or model_confidence > 1.0:
            cloudlog.warning(f"Invalid model confidence value in update_from_model_accuracy: {model_confidence}, clamping to valid range")
            model_confidence = np.clip(model_confidence, 0.0, 1.0)

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

        # Track the update for performance monitoring
        update_time = time.time() - start_time
        self._update_performance_metrics(update_time)

    def _update_performance_metrics(self, update_time: float):
        """
        Update performance metrics for monitoring and optimization.

        Args:
            update_time: Time taken for the last update operation in seconds
        """
        if update_time > self.max_update_time:
            self.max_update_time = update_time

        self.update_time_samples.append(update_time)
        self.total_updates += 1

        # Log performance statistics periodically
        if self.total_updates % 1000 == 0:  # Log every 1000 updates
            avg_update_time = np.mean(self.update_time_samples) if self.update_time_samples else 0
            cloudlog.info(f"Self-Learning Performance - Updates: {self.total_updates}, "
                         f"Avg time: {avg_update_time*1000:.2f}ms, "
                         f"Max time: {self.max_update_time*1000:.2f}ms")
    
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

        # Apply speed-dependent adjustments based on learned parameters (replaces hardcoded speed factor)
        speed_compensation = (self.adaptive_params['speed_compensation_base'] +
                             self.adaptive_params['speed_compensation_rate'] * (v_ego - 15.0))
        # Apply reasonable limits to prevent extreme adjustments
        speed_compensation = max(0.8, min(1.2, speed_compensation))
        adjusted_curvature *= speed_compensation

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
        road_type = experience['road_type']
        model_error = experience.get('model_prediction_error', curvature_error)

        # Calculate adaptive learning rate based on experience and context
        adaptive_lr = self._calculate_adaptive_learning_rate(v_ego, road_type, model_error)

        # Adjust lateral control factor based on systematic errors
        # If desired is consistently different from actual, adjust the scaling
        if abs(curvature_error) > 0.01:  # Significant error
            # Update lateral control factor slowly
            current_factor = self.adaptive_params['lateral_control_factor']

            # Use context-aware adjustment that considers road type and error characteristics
            road_factor = self._get_road_type_factor(road_type)
            adjustment = -curvature_error * adaptive_lr * road_factor

            new_factor = current_factor + adjustment

            # Constrain factor to reasonable range
            new_factor = max(0.7, min(1.3, new_factor))
            self.adaptive_params['lateral_control_factor'] = new_factor

        # Adjust curvature bias if there's a systematic offset
        if abs(curvature_error) > 0.005:
            current_bias = self.adaptive_params['curvature_bias']
            # Use adaptive learning rate for bias as well
            bias_adjustment = -curvature_error * adaptive_lr * 0.5  # Slower bias adjustment
            new_bias = current_bias + bias_adjustment
            self.adaptive_params['curvature_bias'] = new_bias

        # Adjust speed compensation parameters based on context
        if abs(curvature_error) > 0.02 and v_ego > 5.0:  # Only adjust at meaningful speeds
            current_base = self.adaptive_params['speed_compensation_base']
            current_rate = self.adaptive_params['speed_compensation_rate']

            # Adjust base compensation
            base_adjustment = -curvature_error * adaptive_lr * 0.2
            new_base = current_base + base_adjustment
            self.adaptive_params['speed_compensation_base'] = max(0.8, min(1.2, new_base))

            # Adjust rate based on speed-dependent error patterns
            speed_error_factor = (v_ego - 15.0) / 15.0  # Normalized speed deviation from 15 m/s
            rate_adjustment = -curvature_error * adaptive_lr * 0.1 * speed_error_factor
            new_rate = current_rate + rate_adjustment
            self.adaptive_params['speed_compensation_rate'] = max(-0.01, min(0.01, new_rate))

        self.learning_samples += 1
        self._save_adaptive_params_if_needed()

    def _calculate_adaptive_learning_rate(self, v_ego: float, road_type: str, model_error: float) -> float:
        """
        Calculate adaptive learning rate based on current conditions.

        Args:
            v_ego: Vehicle speed
            road_type: Current road type classification
            model_error: Current model prediction error

        Returns:
            Adaptive learning rate
        """
        base_lr = self.learning_rate

        # Increase learning rate for high model errors to adapt faster
        error_factor = min(2.0, 1.0 + abs(model_error) * 10.0)  # Up to 2x for high errors

        # Reduce learning rate at high speeds for safety
        speed_factor = max(0.5, min(1.0, 1.0 - (max(0, v_ego - 30) * 0.01)))  # Reduce above 30 m/s

        # Adjust for road type
        road_factors = {
            'low_speed_urban': 1.2,  # More learning in urban areas
            'city_roads': 1.0,
            'highway_entry': 0.9,
            'highway': 0.8   # Less learning on highways for safety
        }
        road_factor = road_factors.get(road_type, 1.0)

        adaptive_lr = base_lr * error_factor * speed_factor * road_factor
        return min(0.02, adaptive_lr)  # Cap at 0.02 to prevent excessive changes

    def _get_road_type_factor(self, road_type: str) -> float:
        """
        Get road type factor for parameter adjustment.

        Args:
            road_type: Current road type classification

        Returns:
            Factor to scale adjustments by road type
        """
        factors = {
            'low_speed_urban': 1.2,  # More aggressive adjustments in urban
            'city_roads': 1.0,
            'highway_entry': 0.9,
            'highway': 0.8   # More conservative on highways
        }
        return factors.get(road_type, 1.0)
    
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
            # Even if we don't do the full update, check for reset requests
            self.check_for_reset_request()
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

        # Check for reset request
        self.check_for_reset_request()

        self.last_update_time = current_time

        # Log learning statistics periodically
        if self.learning_samples % 50 == 0:
            cloudlog.info(f"Self-Learning Stats - Factor: {self.adaptive_params['lateral_control_factor']:.3f}, "
                         f"Bias: {self.adaptive_params['curvature_bias']:.5f}, "
                         f"Samples: {self.learning_samples}")
    
    def _regularize_parameters(self):
        """
        Apply regularization to prevent parameters from drifting too far from baseline.
        Improved regularization with more nuanced approach to balance safety with beneficial learning.
        """
        # Regularize lateral control factor towards 1.0 with balanced approach
        factor = self.adaptive_params['lateral_control_factor']
        if abs(factor - 1.0) > 0.02:  # Lower threshold for regularization (2% instead of 5%)
            regularization_factor = 0.995  # Slower regularization for beneficial learning
            if abs(factor - 1.0) > 0.25:  # More than 25% deviation - still aggressive for safety
                # Strong regularization to prevent dangerous drift
                regularization_factor = 0.97  # Faster regularization for large deviations
            self.adaptive_params['lateral_control_factor'] = (
                regularization_factor * factor + (1 - regularization_factor) * 1.0
            )

        # Regularize curvature bias towards 0.0 with more nuanced approach
        bias = self.adaptive_params['curvature_bias']
        if abs(bias) > 0.001:  # Lower threshold for regularization
            # Adaptive regularization that's gentler for smaller biases
            if abs(bias) > 0.01:  # For larger biases, more aggressive regularization
                regularization_factor = 0.99  # Faster regularization
            else:
                regularization_factor = 0.995  # Slower regularization for small beneficial adjustments
            self.adaptive_params['curvature_bias'] *= regularization_factor

        # Regularize speed compensation parameters with balanced approach
        speed_base = self.adaptive_params['speed_compensation_base']
        if abs(speed_base - 1.0) > 0.02:  # Regularize base speed compensation toward 1.0
            self.adaptive_params['speed_compensation_base'] = (
                0.995 * speed_base + 0.005 * 1.0  # Balanced regularization
            )

        speed_rate = self.adaptive_params['speed_compensation_rate']
        if abs(speed_rate) > 0.0005:  # Regularize speed rate toward 0 (meaning no speed dependency)
            # Apply balanced regularization to reduce speed dependency over time unless needed
            self.adaptive_params['speed_compensation_rate'] *= 0.995  # Balanced regularization
    
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
            'reaction_time_compensation': 0.2,
            'speed_compensation_base': 1.0,
            'speed_compensation_rate': 0.0
        }
        self.learning_samples = 0
        self.experience_buffer.clear()
        self.intervention_buffer.clear()
        self.model_accuracy_history.clear()
        self.save_learning_state()
        cloudlog.info("Learning state reset to initial values")

    def check_for_reset_request(self):
        """
        Check for reset request from parameters and perform reset if requested.
        This provides a user-facing mechanism to reset learned parameters.
        """
        try:
            # Check if reset parameter has been set
            reset_request = self.params.get("ResetSelfLearning", encoding='utf-8')
            if reset_request and reset_request.lower() == "1":
                cloudlog.info("Reset request detected, resetting self-learning state")
                self.reset_learning_state()
                # Clear the reset parameter so it doesn't trigger again
                self.params.delete("ResetSelfLearning")
        except Exception as e:
            cloudlog.warning(f"Error checking for reset request: {e}")