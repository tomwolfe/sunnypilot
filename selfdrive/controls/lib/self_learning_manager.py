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
from typing import Optional
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
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
        self.base_learning_rate = 0.01
        self.learning_rate = self.base_learning_rate  # Initialize the learning_rate
        self.confidence_threshold = 0.7
        self.intervention_threshold = 0.5  # Amount of steering correction to trigger learning
        self.min_adjustment_threshold = 0.001  # Minimum adjustment to apply
        # Track performance metrics
        self.performance_start_time = time.monotonic()
        self.total_updates = 0
        self.max_update_time = 0.0
        self.update_time_samples: deque[float] = deque(maxlen=100)  # Track recent update times
        # Memory for storing recent experiences
        self.experience_buffer: deque[dict] = deque(maxlen=1000)  # Store recent driving experiences
        self.intervention_buffer: deque[dict] = deque(maxlen=100)  # Store interventions
        self.model_accuracy_history: deque[dict] = deque(maxlen=500)  # Track model prediction accuracy
        # Enhanced adaptive parameters with more sophisticated learning
        self.adaptive_params = {
            'lateral_control_factor': 1.0,  # Scaling factor for lateral control
            'curvature_bias': 0.0,  # Bias adjustment to desired curvature
            'acceleration_factor': 1.0,  # Scaling factor for longitudinal acceleration
            'reaction_time_compensation': 0.2,  # Time compensation in seconds
        }
        # Learning context tracking
        self.learning_context = {
            'road_type': 'unknown',
            'weather_condition': 'clear',  # Could be 'clear', 'rainy', 'snowy', 'foggy'
            'traffic_density': 'low',      # Could be 'low', 'medium', 'high'
            'time_of_day': 'day',          # Could be 'day', 'dusk', 'night'
            'road_surface': 'dry',         # Could be 'dry', 'wet', 'icy'
        }
        # Learning state tracking
        self.last_update_time = time.monotonic()
        self.update_interval = 5.0  # Update parameters every 5 seconds
        self.learning_samples = 0
        self.total_samples = 0
        # Enhanced experience weighting system
        self.experience_weights = {  # Weight for different experience types
            'intervention': 1.0,
            'model_accuracy': 0.5,
            'normal_driving': 0.1
        }
        # Initialize from saved parameters if available
        self.load_adaptive_params()
        # Enhanced monitoring for over-adaptation and system reliability
        try {
            from enhanced_self_learning_monitoring import EnhancedSelfLearningMonitor, EnhancedSafetyValidator
            self.enhanced_monitor = EnhancedSelfLearningMonitor()
            self.enhanced_safety_validator = EnhancedSafetyValidator()  # Create a shared instance
        } except ImportError {
            cloudlog.warning("Enhanced monitoring module not available, using basic monitoring")
            self.enhanced_monitor = None
            self.enhanced_safety_validator = None
        }
        self._prev_validation_curvatures: list[float] = []
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
        start_time = time.monotonic()
        # Detect if driver is overriding the system
        steering_pressed = CS.steeringPressed
        if not steering_pressed:
            # Track the update for performance monitoring
            update_time = time.monotonic() - start_time
            self._update_performance_metrics(update_time)
            return
        # Validate model confidence to ensure it's within expected range
        if model_confidence < 0.0 or model_confidence > 1.0:
            cloudlog.warning(f"Invalid model confidence value: {model_confidence}, clamping to valid range")
            model_confidence = np.clip(model_confidence, 0.0, 1.0)
        # Calculate the difference between desired and actual
        curvature_error = desired_curvature - actual_curvature
        torque_magnitude = abs(steering_torque)
        # Update learning context from current driving conditions
        self._update_learning_context(CS, v_ego, desired_curvature)
        # Context-aware learning: only learn when it's likely a corrective action
        # Check if there's a significant model prediction error and driver is correcting it
        model_error_significant = model_prediction_error is not None and abs(model_prediction_error) > 0.02
        correction_direction_matches = (model_prediction_error is not None and
                                       np.sign(model_prediction_error) == np.sign(steering_torque))
        # Additional checks for context-aware learning
        high_model_error = model_error_significant and abs(model_prediction_error) > 0.05
        low_model_confidence = model_confidence < self.confidence_threshold  # Use actual model confidence instead of hardcoded False
        driver_correction = torque_magnitude > self.intervention_threshold
        # Enhanced context: consider road type and conditions
        current_road_type = self._classify_road_type(v_ego, abs(desired_curvature))
        is_highway = current_road_type == 'highway'
        is_urban = current_road_type in ['low_speed_urban', 'city_roads']
        # Adjust learning conditions based on road type and safety considerations
        # Highway driving typically requires more conservative learning
        highway_learning_factor = 0.5 if is_highway else 1.0
        urban_learning_factor = 1.2 if is_urban else 1.0
        # Only learn if it's likely a corrective action (high model error or correction in the right direction) and model confidence is sufficient
        should_learn = (high_model_error or (model_error_significant and correction_direction_matches)) and driver_correction and not low_model_confidence
        should_learn = should_learn and (highway_learning_factor > 0.6 or urban_learning_factor > 1.0)  # More conservative on highways
        # Additional safety check - don't learn if the model is performing well
        if model_error_significant and abs(curvature_error) < 0.01 and torque_magnitude < 0.5:
            should_learn = False
        # Safety check - don't learn during dangerous maneuvers
        if v_ego > 30.0 and abs(desired_curvature) > 0.05:  # High speed with high curvature
            should_learn = False
            cloudlog.warning(f"Skipping learning at vEgo={v_ego:.2f}, curvature={desired_curvature:.4f} due to safety concerns")
        if should_learn:
            experience = {
                'timestamp': time.monotonic(),
                'desired_curvature': desired_curvature,
                'actual_curvature': actual_curvature,
                'curvature_error': curvature_error,
                'steering_torque': steering_torque,
                'v_ego': v_ego,
                'model_prediction_error': model_prediction_error,
                'model_confidence': model_confidence,
                'road_type': current_road_type,
                'intervention_type': 'corrective_steering',
                'context': self.learning_context.copy(),  # Include current context
                'learning_factor': max(highway_learning_factor, urban_learning_factor),  # Include learning factor
                'original_params': self.adaptive_params.copy()  # Store original params for comparison
            }
            self.intervention_buffer.append(experience)
            self.experience_buffer.append(experience)
            # Adjust parameters based on intervention with context-aware learning rates
            self._adapt_from_intervention(experience)
            # Log detailed information about the learning adjustment
            if self.learning_samples % 25 == 0:  # Log every 25 learning events
                param_changes = {}
                for key in self.adaptive_params:
                    original_value = experience['original_params'][key]
                    new_value = self.adaptive_params[key]
                    change = abs(new_value - original_value)
                    param_changes[key] = {
                        'from': original_value,
                        'to': new_value,
                        'change': change
                    }
                cloudlog.info(f"Self-Learning Adjustment - Curvature Error: {curvature_error:.4f}, "
                             f"Learning Rate: {self.base_learning_rate:.4f}, "
                             f"Context: {current_road_type}, "
                             f"Parameter Changes: {param_changes}")
        # Track the update for performance monitoring
        update_time = time.monotonic() - start_time
        self._update_performance_metrics(update_time)
    def _update_learning_context(self, CS, v_ego: float, desired_curvature: float):
        """
        Update the learning context based on current driving conditions.
        Args:
            CS: CarState message
            v_ego: Vehicle speed
            desired_curvature: Current desired curvature
        """
        # Validate inputs to prevent invalid context updates
        if v_ego < 0 or abs(desired_curvature) > 0.5:  # Max reasonable curvature
            cloudlog.warning(f"Invalid input values for context update: v_ego={v_ego}, curvature={desired_curvature}")
            return
        # Update road type context
        old_road_type = self.learning_context.get('road_type', 'unknown')
        self.learning_context['road_type'] = self._classify_road_type(v_ego, abs(desired_curvature))
        # Log if road type changed unexpectedly
        if old_road_type != self.learning_context['road_type']:
            cloudlog.debug(f"Road type changed from {old_road_type} to {self.learning_context['road_type']} "
                          f"at v_ego={v_ego}, curvature={desired_curvature}")
        # Update traffic density based on radar/lidar data if available
        old_traffic_density = self.learning_context.get('traffic_density', 'low')
        # This is a simplified version - actual implementation would use radar data
        if hasattr(CS, 'leadOne') and CS.leadOne is not None:
            if CS.leadOne.dRel < 50:  # Lead car within 50m
                self.learning_context['traffic_density'] = 'high' if CS.leadOne.dRel < 20 else 'medium'
            else:
                self.learning_context['traffic_density'] = 'low'
        elif hasattr(CS, 'leadOne') and CS.leadOne is None:
            # No lead car data available
            self.learning_context['traffic_density'] = 'unknown'
        # Log if traffic density changed
        if old_traffic_density != self.learning_context.get('traffic_density'):
            cloudlog.debug(f"Traffic density changed from {old_traffic_density} to {self.learning_context.get('traffic_density')}")
        # Update weather based on environmental sensors if available
        if hasattr(CS, 'rainRadar') and CS.rainRadar > 0.5:  # If rain sensor is available and detects rain
            self.learning_context['weather_condition'] = 'rainy'
        elif hasattr(CS, 'steerOverride') and CS.steerOverride:  # Driver taking manual control might indicate challenging conditions
            # Don't change weather condition based on steering override alone
            pass
        # Update time of day (simplified - would use actual time and GPS location in real implementation)
        # For now, we'll use a simple heuristic
        if v_ego > 10 and hasattr(CS, 'gpsPlausible') and CS.gpsPlausible:
            # This is a simplified approach - real implementation would use actual time and location
            pass  # Placeholder for real time of day detection
        # Update road surface condition based on driving behavior and environment with more validation
        old_road_surface = self.learning_context.get('road_surface', 'dry')
        if hasattr(CS, 'wheelSpeeds') and CS.wheelSpeeds is not None and abs(v_ego) > 1.0:
            try {
                # Detect potential slippery conditions based on wheel speed vs vehicle speed
                # This is a simplified approach - real implementation would use more sophisticated physics
                if (hasattr(CS, 'brakePressed') and CS.brakePressed and
                    v_ego > 5.0 and abs(CS.aEgo) > 3.0 and abs(CS.vEgo) > 5.0):  # Hard braking with high deceleration at meaningful speed
                    # Check for wheel slip by comparing vehicle speed to wheel speeds
                    if (hasattr(CS.wheelSpeeds, 'fl') and hasattr(CS.wheelSpeeds, 'fr') and
                        CS.wheelSpeeds.fl is not None and CS.wheelSpeeds.fr is not None) {
                        avg_wheel_speed = (CS.wheelSpeeds.fl + CS.wheelSpeeds.fr) / 2.0
                        speed_diff = abs(v_ego - avg_wheel_speed)
                        if speed_diff > 2.0 and abs(CS.aEgo) > 3.0 {  // Significant difference during hard braking
                            self.learning_context['road_surface'] = 'icy'  // High probability of slippery surface
                        } else if speed_diff > 1.0 and abs(CS.aEgo) > 3.0 {
                            self.learning_context['road_surface'] = 'wet'  // Moderate probability
                        }
                    }
                }
            } catch (AttributeError, TypeError) {
                cloudlog.warning(f"Error checking wheel speeds for road surface detection: {e}")
            }
        # Log if road surface changed
        if old_road_surface != self.learning_context.get('road_surface'):
            cloudlog.debug(f"Road surface changed from {old_road_surface} to {self.learning_context.get('road_surface')}")
    def _calculate_contextual_learning_rate(self, experience: dict) -> float:
        """
        Calculate learning rate based on contextual factors.
        Args:
            experience: Experience record containing contextual information
        Returns:
            Contextually adjusted learning rate
        """
        context = experience.get('context', {})
        learning_factor = experience.get('learning_factor', 1.0)
        # Start with base learning rate
        learning_rate = self.base_learning_rate * learning_factor
        # Adjust based on weather conditions
        if context.get('weather_condition') == 'rainy':
            learning_rate *= 0.7  # More conservative in rain
        elif context.get('weather_condition') == 'snowy':
            learning_rate *= 0.5  # Very conservative in snow
        # Adjust based on traffic density
        if context.get('traffic_density') == 'high':
            learning_rate *= 0.8  # More conservative in heavy traffic
        elif context.get('traffic_density') == 'medium':
            learning_rate *= 0.9
        # Adjust based on time of day
        if context.get('time_of_day') == 'night':
            learning_rate *= 0.8  # More conservative at night
        elif context.get('time_of_day') == 'dusk':
            learning_rate *= 0.9
        # Adjust based on road surface
        if context.get('road_surface') == 'wet':
            learning_rate *= 0.7
        elif context.get('road_surface') == 'icy':
            learning_rate *= 0.4
        # Ensure learning rate stays within reasonable bounds
        return min(0.02, max(0.001, learning_rate))  # Clamp to reasonable range
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
        start_time = time.monotonic()
        # Validate model confidence to ensure it's within expected range
        if model_confidence < 0.0 or model_confidence > 1.0:
            cloudlog.warning(f"Invalid model confidence value in update_from_model_accuracy: {model_confidence}, clamping to valid range")
            model_confidence = np.clip(model_confidence, 0.0, 1.0)
        # Calculate prediction error
        prediction_error = abs(desired_curvature - actual_curvature)
        # Track model accuracy over time
        accuracy_record = {
            'timestamp': time.monotonic(),
            'error': prediction_error,
            'confidence': model_confidence,
            'v_ego': v_ego,
            'adjustment_needed': prediction_error > 0.01  # Significant error threshold
        }
        self.model_accuracy_history.append(accuracy_record)
        # Store experience if there's a meaningful discrepancy
        if accuracy_record['adjustment_needed'] and model_confidence > self.confidence_threshold:
            experience = {
                'timestamp': time.monotonic(),
                'type': 'model_accuracy',
                'desired_curvature': desired_curvature,
                'actual_curvature': actual_curvature,
                'prediction_error': prediction_error,
                'model_confidence': model_confidence,
                'v_ego': v_ego,
                'original_params': self.adaptive_params.copy()  # Store original params for comparison
            }
            self.experience_buffer.append(experience)
        # Log accuracy metrics periodically for monitoring
        if len(self.model_accuracy_history) % 100 == 0:
            recent_errors = [rec['error'] for rec in list(self.model_accuracy_history)[-50:]]
            if recent_errors:
                avg_error = np.mean(recent_errors)
                std_error = np.std(recent_errors)
                cloudlog.info(f"Model Accuracy Monitoring - Last 50 samples: "
                             f"Avg Error: {avg_error:.5f}, Std: {std_error:.5f}, "
                             f"Current Error: {prediction_error:.5f}, "
                             f"Confidence: {model_confidence:.3f}")
        # Track the update for performance monitoring
        update_time = time.monotonic() - start_time
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
        # Apply learned adjustments with enhanced context awareness
        adjusted_curvature = original_curvature * self.adaptive_params['lateral_control_factor']
        adjusted_curvature += self.adaptive_params['curvature_bias']
        return adjusted_curvature
    def _calculate_max_safe_curvature(self, v_ego: float) -> float:
        """
        Calculate maximum safe curvature based on vehicle speed.
        Args:
            v_ego: Vehicle speed in m/s
        Returns:
            Maximum safe curvature in 1/m
        """
        if v_ego < 0.1:  # Very low speed
            return 0.5  # Higher curvature allowed at low speeds
        # Use the enhanced monitor for vehicle-specific limits if available
        max_lat_accel = 2.5  # Default value
        if self.enhanced_monitor:
            max_lat_accel = self.enhanced_monitor.get_updated_lateral_acceleration_limit()
        # Based on max lateral acceleration = v^2 * curvature
        # So curvature = max_lat_accel / v^2
        max_curvature = max_lat_accel / (v_ego * v_ego) if v_ego > 0.1 else 0.5
        return min(max_curvature, 0.5)  # Cap at 0.5 for additional safety
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
        # Apply speed-dependent adjustment for safer acceleration at high speeds
        if v_ego > 20.0:  # Above ~72 km/h
            speed_adjustment = max(0.7, min(1.0, 1.0 - (v_ego - 20.0) * 0.01))  # More conservative at high speed
            adjusted_accel *= speed_adjustment
        # Apply maximum acceleration limits based on context
        max_accel = 3.0  # Base max acceleration
        if self.learning_context.get('road_surface') in ['wet', 'icy']:
            max_accel *= 0.6  # More conservative in bad weather
        elif self.learning_context.get('traffic_density') in ['high', 'medium']:
            max_accel *= 0.8  # More conservative in traffic
        # Also limit deceleration (braking)
        max_braking = -4.0  # Base max deceleration
        if self.learning_context.get('road_surface') in ['wet', 'icy']:
            max_braking *= 0.5  # Gentler braking in bad weather
        adjusted_accel = np.clip(adjusted_accel, max_braking, max_accel)
        return adjusted_accel
    def _adapt_from_intervention(self, experience: dict):
        """
        Adapt system parameters based on a driver intervention experience.
        Args:
            experience: Experience record containing intervention details
        """
        curvature_error = experience['curvature_error']
        road_type = experience['road_type']
        # Calculate contextual learning rate based on experience and context
        adaptive_lr = self._calculate_contextual_learning_rate(experience)
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
        # Adjust acceleration factor based on consistency of interventions
        if abs(curvature_error) > 0.02:
            current_accel_factor = self.adaptive_params['acceleration_factor']
            # Adjust acceleration factor based on error patterns
            accel_adjustment = -curvature_error * adaptive_lr * 0.1
            new_accel_factor = current_accel_factor + accel_adjustment
            # Constrain factor to reasonable range
            new_accel_factor = max(0.7, min(1.3, new_accel_factor))
            self.adaptive_params['acceleration_factor'] = new_accel_factor
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
    def periodic_update(self, CS=None):
        """
        Perform periodic learning updates based on accumulated data.
        This should be called periodically to update parameters based on recent experiences.
        Args:
            CS: CarState object for additional vehicle context (optional)
        """
        current_time = time.monotonic()
        if current_time - self.last_update_time < self.update_interval:
            # Even if we don't do the full update, check for reset requests
            self.check_for_reset_request()
            return
        # Update learning based on recent experiences
        if len(self.model_accuracy_history) > 10:
            # Calculate average prediction error
            recent_errors = [rec['error'] for rec in list(self.model_accuracy_history)[-20:]]
            avg_error = np.mean(recent_errors) if recent_errors else 0
            # Calculate error trend (improving vs getting worse)
            if len(recent_errors) >= 5:
                recent_error = np.mean(recent_errors[-5:])  # Last 5 errors
                older_error = np.mean(recent_errors[:-5] if len(recent_errors) > 5 else recent_errors)  # Prior errors
                error_trend = recent_error - older_error  # Negative = improving, Positive = getting worse
            else:
                error_trend = 0.0
            # Adjust parameters based on overall accuracy and trend
            if avg_error > 0.025:  # High error threshold
                # Increase learning rate temporarily for faster adaptation
                self.base_learning_rate = min(0.02, self.base_learning_rate * 1.2)
            elif avg_error < 0.015 and error_trend <= 0:  # Low error and improving
                # Decrease learning rate to stabilize as we're doing well
                self.base_learning_rate = max(0.005, self.base_learning_rate * 0.85)
            elif error_trend > 0.005:  # Error is getting significantly worse
                # Increase learning rate to adapt quickly to changing conditions
                self.base_learning_rate = min(0.025, self.base_learning_rate * 1.3)
        # Regularize parameters to prevent excessive drift
        self._regularize_parameters()
        # Adaptive update interval based on learning activity
        if self.learning_samples > 0 and self.total_updates > 0:
            # If we're learning actively, consider adjusting update frequency
            recent_learning_rate = self.learning_samples / self.total_updates
            if recent_learning_rate > 0.1:  # More than 10% of cycles involve learning
                # Increase update frequency when learning is active
                self.update_interval = max(2.0, 5.0 - (recent_learning_rate * 10.0))
            else:
                # Reduce update frequency when learning is sparse
                self.update_interval = min(10.0, 5.0 + (0.1 - recent_learning_rate) * 5.0)
        # Check for reset request
        self.check_for_reset_request()
        self.last_update_time = current_time
        # Log learning statistics periodically with enhanced detail
        if self.learning_samples % 50 == 0:
            cloudlog.info(f"Self-Learning Stats - Factor: {self.adaptive_params['lateral_control_factor']:.3f}, "
                         f"Bias: {self.adaptive_params['curvature_bias']:.5f}, "
                         f"Weather: {self.adaptive_params['weather_adaptation_factor']:.3f}, "
                         f"Traffic: {self.adaptive_params['traffic_density_factor']:.3f}, "
                         f"Samples: {self.learning_samples}, "
                         f"Base_LR: {self.base_learning_rate:.4f}, "
                         f"Context: {self.learning_context}")
        # Comprehensive monitoring and logging for self-learning system
        self._comprehensive_monitoring()
        # Enhanced monitoring for over-adaptation and system reliability
        if self.enhanced_monitor:
            # Monitor for over-adaptation patterns
            over_adaptation_result = self.enhanced_monitor.monitor_over_adaptation(
                self.adaptive_params, self.learning_context
            )
            # If over-adaptation is detected, take corrective actions
            if over_adaptation_result['over_adaptation_detected']:
                cloudlog.warning(f"Over-adaptation detected: {', '.join(over_adaptation_result['triggering_conditions'])}")
                # Reduce learning rate temporarily
                self.base_learning_rate *= 0.7
                cloudlog.info(f"Reduced learning rate to {self.base_learning_rate:.4f} due to over-adaptation detection")
                # Apply more aggressive regularization
                self._regularize_parameters(over_adaptation=True)
                # Log suggested actions
                if over_adaptation_result['suggested_actions']:
                    for action in over_adaptation_result['suggested_actions']:
                        cloudlog.info(f"Suggested action: {action}")
            # Update vehicle calibration
            if CS and hasattr(CS, 'vEgo') and hasattr(CS, 'aEgo'):
                v_ego = CS.vEgo if CS.vEgo is not None else 0
                # Calculate lateral acceleration if we have the needed data
                current_curvature = self.adaptive_params.get('curvature_bias', 0.0)
                lateral_accel = v_ego * v_ego * current_curvature if v_ego > 0.1 else 0
                self.enhanced_monitor.update_vehicle_calibration(v_ego, current_curvature, lateral_accel)
        # Track computational performance
        if self.enhanced_monitor:
            start_time = time.monotonic()
            computation_time = self.enhanced_monitor.track_computational_performance(start_time)
    def _comprehensive_monitoring(self):
        """
        Comprehensive monitoring and logging for the self-learning system to track
        performance, parameter drift, learning effectiveness, and safety metrics.
        """
        # Calculate various metrics for monitoring
        monitoring_data = {
            'timestamp': time.monotonic(),
            'learning_samples': self.learning_samples,
            'total_updates': self.total_updates,
            'base_learning_rate': self.base_learning_rate,
            'update_interval': self.update_interval,
            'adaptive_params': self.adaptive_params.copy(),
            'context': self.learning_context.copy(),
            'performance_metrics': {}
        }
        # Calculate performance metrics
        if len(self.model_accuracy_history) > 0:
            recent_accuracy = list(self.model_accuracy_history)[-min(50, len(self.model_accuracy_history)):]
            if recent_accuracy:
                errors = [rec['error'] for rec in recent_accuracy]
                monitoring_data['performance_metrics']['avg_error'] = np.mean(errors)
                monitoring_data['performance_metrics']['std_error'] = np.std(errors)
                monitoring_data['performance_metrics']['min_error'] = np.min(errors)
                monitoring_data['performance_metrics']['max_error'] = np.max(errors)
                monitoring_data['performance_metrics']['error_trend'] = (
                    np.mean(errors[-10:]) - np.mean(errors[:10])
                    if len(errors) > 20 else 0.0
                )
        # Calculate parameter stability metrics
        param_stability = {
            'lateral_control_factor': abs(self.adaptive_params['lateral_control_factor'] - 1.0),
            'curvature_bias': abs(self.adaptive_params['curvature_bias']),
            'acceleration_factor': abs(self.adaptive_params['acceleration_factor'] - 1.0),
            'reaction_time_compensation': abs(self.adaptive_params['reaction_time_compensation'] - 0.2)
        }
        monitoring_data['param_stability'] = param_stability
        monitoring_data['max_param_drift'] = max(param_stability.values())
        # Calculate learning effectiveness
        if self.total_updates > 0:
            learning_efficiency = self.learning_samples / max(1, self.total_updates)
            monitoring_data['learning_efficiency'] = learning_efficiency
        # Log detailed monitoring data periodically (less frequently than basic stats)
        if self.learning_samples % 200 == 0:  # Log detailed metrics every 200 learning samples
            cloudlog.info(f"Self-Learning Monitoring - "
                         f"Efficiency: {monitoring_data.get('learning_efficiency', 0):.3f}, "
                         f"Avg Error: {monitoring_data['performance_metrics'].get('avg_error', 0):.5f}, "
                         f"Error Trend: {monitoring_data['performance_metrics'].get('error_trend', 0):.5f}, "
                         f"Max Param Drift: {monitoring_data['max_param_drift']:.4f}, "
                         f"Learning Rate: {monitoring_data['base_learning_rate']:.4f}")
        # Log warnings if parameters drift too far from baseline
        drift_threshold = 0.5  # 50% drift from baseline
        for param_name, drift in param_stability.items():
            if drift > drift_threshold:
                cloudlog.warning(f"Self-Learning Parameter Drift Alert - {param_name}: {drift:.4f} "
                               f"(current: {self.adaptive_params[param_name]:.4f})")
        # Log if learning efficiency is unexpectedly high or low
        if 'learning_efficiency' in monitoring_data:
            eff = monitoring_data['learning_efficiency']
            if eff > 0.8:  # Very high learning frequency might indicate instability
                cloudlog.warning(f"Self-Learning High Activity Alert - Learning Efficiency: {eff:.3f}, "
                               f"Learning may be too frequent, consider adjusting thresholds")
            elif eff < 0.01 and self.total_updates > 100:  # Very low might indicate no learning happening
                cloudlog.info(f"Self-Learning Low Activity - Learning Efficiency: {eff:.3f}, "
                            f"May need to adjust intervention thresholds")
        # Store monitoring data for potential external analysis
        if not hasattr(self, '_monitoring_history'):
            self._monitoring_history = []
        self._monitoring_history.append(monitoring_data)
        # Keep only recent history to prevent memory bloat
        self._monitoring_history = self._monitoring_history[-500:]  # Keep last 500 monitoring entries
    def _regularize_parameters(self, over_adaptation=False):
        """
        Apply regularization to prevent parameters from drifting too far from baseline.
        Improved regularization with more nuanced approach to balance safety with beneficial learning.
        Args:
            over_adaptation: If True, apply more aggressive regularization to address over-adaptation
        """
        # Determine regularization factors based on whether we're dealing with over-adaptation
        if over_adaptation:
            # More aggressive regularization when over-adaptation is detected
            slow_regularization_factor = 0.98  # Faster regularization
            fast_regularization_factor = 0.95  # Much faster for dangerous deviations
            bias_slow_reg_factor = 0.98
            bias_fast_reg_factor = 0.95
        else:
            # Normal balanced regularization
            slow_regularization_factor = 0.995  # Slower regularization for beneficial learning
            fast_regularization_factor = 0.97   # Faster regularization for large deviations
            bias_slow_reg_factor = 0.995
            bias_fast_reg_factor = 0.99
        # Regularize lateral control factor towards 1.0 with balanced approach
        factor = self.adaptive_params['lateral_control_factor']
        if abs(factor - 1.0) > 0.02:  # Lower threshold for regularization (2% instead of 5%)
            if abs(factor - 1.0) > 0.25:  # More than 25% deviation - aggressive regularization
                # Strong regularization to prevent dangerous drift
                regularization_factor = fast_regularization_factor
            else:
                regularization_factor = slow_regularization_factor
            self.adaptive_params['lateral_control_factor'] = (
                regularization_factor * factor + (1 - regularization_factor) * 1.0
            )
        # Regularize curvature bias towards 0.0 with more nuanced approach
        bias = self.adaptive_params['curvature_bias']
        if abs(bias) > 0.001:  # Lower threshold for regularization
            # Adaptive regularization that's gentler for smaller biases
            if abs(bias) > 0.01 or over_adaptation:  # For larger biases, or if over-adaptation detected
                regularization_factor = bias_fast_reg_factor  # More aggressive regularization
            else:
                regularization_factor = bias_slow_reg_factor  # Gentle regularization for small adjustments
            self.adaptive_params['curvature_bias'] *= regularization_factor
        # Regularize acceleration factor towards 1.0 with balanced approach
        accel_factor = self.adaptive_params['acceleration_factor']
        if abs(accel_factor - 1.0) > 0.02:  # Lower threshold for regularization (2% instead of 5%)
            if abs(accel_factor - 1.0) > 0.25:  # More than 25% deviation - aggressive regularization
                # Strong regularization to prevent dangerous drift
                regularization_factor = fast_regularization_factor
            else:
                regularization_factor = slow_regularization_factor
            self.adaptive_params['acceleration_factor'] = (
                regularization_factor * accel_factor + (1 - regularization_factor) * 1.0
            )
        # Regularize reaction time compensation towards default 0.2
        reaction_time = self.adaptive_params['reaction_time_compensation']
        if abs(reaction_time - 0.2) > 0.05:  # Regularize if significantly different from default
            reg_factor = 0.99  # Gentle regularization
            self.adaptive_params['reaction_time_compensation'] = (
                reg_factor * reaction_time + (1 - reg_factor) * 0.2
            )
    def save_learning_state(self):
        """
        Save the current learning state to persistent storage.
        """
        try {
            learning_state = {
                'adaptive_params': self.adaptive_params,
                'learning_rate': self.learning_rate,
                'learning_samples': self.learning_samples,
                'timestamp': time.monotonic()
            }
            self.params.put("SelfLearningState", str(learning_state))
            cloudlog.info("Learning state saved")
        } catch (Exception e) {
            cloudlog.error(f"Failed to save learning state: {e}")
        }
    def load_adaptive_params(self):
        """
        Load adaptive parameters from persistent storage if available.
        """
        try {
            saved_state_str = self.params.get("SelfLearningState")
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
            }
        } catch (Exception e) {
            cloudlog.warning(f"Failed to load learning state: {e}")
        }
    def _save_adaptive_params_if_needed(self):
        """
        Save adaptive parameters periodically to persistent storage.
        """
        if self.learning_samples % 100 == 0:  // Save every 100 learning samples
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
        }
        self.learning_samples = 0
        self.experience_buffer.clear()
        self.intervention_buffer.clear()
        self.model_accuracy_history.clear()
        self.save_learning_state()
        cloudlog.info("Learning state reset to initial values")
    def validate_learned_parameters_safety(self, CS, desired_curvature: float, desired_acceleration: float, v_ego: float) -> dict:
        """
        Validate learned parameters for safety before applying them to vehicle controls.
        This method performs comprehensive safety validation of learned parameters
        to ensure they don't result in unsafe driving behavior with critical safety checks.
        Args:
            CS: Current car state
            desired_curvature: Desired curvature after learning adjustments
            desired_acceleration: Desired acceleration after learning adjustments
            v_ego: Vehicle speed
        Returns:
            dictionary with validation results including safety flags and suggested corrections
        """
        validation_results = {
            'is_safe': True,
            'safety_issues': [],
            'corrected_curvature': desired_curvature,
            'corrected_acceleration': desired_acceleration,
            'confidence_in_safety': 1.0,  # 0.0 to 1.0 scale
            'validation_details': {},
            'critical_violations': [],  # Track critical safety violations
            'validation_passed': True,  # Whether validation passed
            'overrides_applied': False  # Whether any overrides were applied
        }
        # Safety validation for input parameters
        if v_ego < 0:
            validation_results['is_safe'] = False
            validation_results['critical_violations'].append(f"CRITICAL: Invalid vehicle speed: {v_ego} m/s")
            cloudlog.error(f"CRITICAL: Invalid v_ego in safety validation: {v_ego}")
            return validation_results  # Early return for critical input validation failure
        # Use enhanced safety validator if available
        enhanced_validation = None
        if self.enhanced_safety_validator:
            enhanced_validation = self.enhanced_safety_validator.validate_with_computational_efficiency(
                self.adaptive_params, v_ego
            )
            # Check if enhanced validation detected critical violations
            if enhanced_validation and enhanced_validation.get('critical_violations'):
                validation_results['is_safe'] = False
                validation_results['critical_violations'].extend(enhanced_validation['critical_violations'])
                validation_results['safety_issues'].extend(enhanced_validation['safety_issues'])
        # Validate lateral control parameters (curvature) with more stringent checks
        max_safe_curvature = self._calculate_max_safe_curvature(v_ego)
        if abs(desired_curvature) > max_safe_curvature:
            validation_results['is_safe'] = False
            validation_results['critical_violations'].append(f"CRITICAL: Excessive curvature: {desired_curvature:.4f} > {max_safe_curvature:.4f}")
            validation_results['safety_issues'].append(f"Excessive curvature: {desired_curvature:.4f} > {max_safe_curvature:.4f}")
            validation_results['corrected_curvature'] = max(-max_safe_curvature, min(max_safe_curvature, desired_curvature))
            validation_results['overrides_applied'] = True
        # Validate very small curvature values that might indicate invalid calculations
        if abs(desired_curvature) < 0.0001 and abs(CS.aEgo) > 1.0:
            # If vehicle is accelerating/decelerating significantly but curvature is nearly zero,
            # this might indicate a calculation issue
            validation_results['safety_issues'].append(f"Potentially invalid near-zero curvature with significant longitudinal acceleration: {CS.aEgo}")
        # Check for excessive lateral acceleration
        lateral_accel = v_ego * v_ego * desired_curvature if v_ego > 0.1 else 0.0
        max_lateral_accel = 2.5  # m/s^2 or use vehicle-specific limit
        if self.enhanced_monitor:
            max_lateral_accel = self.enhanced_monitor.get_updated_lateral_acceleration_limit()
        # Check for excessive lateral acceleration
        if abs(lateral_accel) > max_lateral_accel:
            validation_results['is_safe'] = False
            validation_results['critical_violations'].append(f"CRITICAL: Excessive lateral acceleration: {lateral_accel:.2f} > {max_lateral_accel:.2f}")
            validation_results['safety_issues'].append(f"Excessive lateral acceleration: {lateral_accel:.2f} > {max_lateral_accel:.2f}")
        # Validate longitudinal control parameters (acceleration) with critical thresholds
        max_accel = 3.0 if v_ego < 15 else 2.0  # Reduce max acceleration at high speed
        max_brake = -4.0 if v_ego < 15 else -3.0  # Reduce max braking at high speed
        if desired_acceleration > max_accel:
            validation_results['is_safe'] = False
            validation_results['critical_violations'].append(f"CRITICAL: Excessive acceleration: {desired_acceleration:.2f} > {max_accel:.2f}")
            validation_results['safety_issues'].append(f"Excessive acceleration: {desired_acceleration:.2f} > {max_accel:.2f}")
            validation_results['corrected_acceleration'] = max_accel
            validation_results['overrides_applied'] = True
        elif desired_acceleration < max_brake:
            validation_results['is_safe'] = False
            validation_results['critical_violations'].append(f"CRITICAL: Excessive braking: {desired_acceleration:.2f} < {max_brake:.2f}")
            validation_results['safety_issues'].append(f"Excessive braking: {desired_acceleration:.2f} < {max_brake:.2f}")
            validation_results['corrected_acceleration'] = max_brake
            validation_results['overrides_applied'] = True
        # Check for parameter drift beyond safe bounds with more critical thresholds
        param_drift_issues = []
        critical_drift_issues = []
        for param_name, current_value in self.adaptive_params.items():
            if param_name == 'curvature_bias':
                # Curvature bias should be within much more restrictive limits - this is very sensitive
                if abs(current_value) > 0.05:  # Much more restrictive threshold
                    critical_drift_issues.append(f"{param_name}: {current_value:.4f}")
                    validation_results['is_safe'] = False
                elif abs(current_value) > 0.02:  # Warning threshold
                    param_drift_issues.append(f"{param_name}: {current_value:.4f}")
            elif param_name == 'lateral_control_factor':
                # Scaling factors should not drift too far from 1.0
                if abs(current_value - 1.0) > 0.7:  # Even more restrictive for this critical parameter
                    critical_drift_issues.append(f"{param_name}: {current_value:.3f}")
                    validation_results['is_safe'] = False
                elif abs(current_value - 1.0) > 0.4:  # Warning threshold
                    param_drift_issues.append(f"{param_name}: {current_value:.3f}")
            elif param_name == 'acceleration_factor':
                # Similar to lateral control
                if abs(current_value - 1.0) > 0.7:
                    critical_drift_issues.append(f"{param_name}: {current_value:.3f}")
                    validation_results['is_safe'] = False
                elif abs(current_value - 1.0) > 0.4:
                    param_drift_issues.append(f"{param_name}: {current_value:.3f}")
            elif param_name in ['weather_adaptation_factor', 'traffic_density_factor']:
                # Adaptation factors should be within reasonable bounds
                if current_value < 0.2 or current_value > 2.0:  # More restrictive bounds
                    critical_drift_issues.append(f"{param_name}: {current_value:.3f}")
                    validation_results['is_safe'] = False
                elif current_value < 0.3 or current_value > 1.5:  # Warning bounds
                    param_drift_issues.append(f"{param_name}: {current_value:.3f}")
        if critical_drift_issues:
            validation_results['critical_violations'].append(f"Critical parameter drift detected: {', '.join(critical_drift_issues)}")
        if param_drift_issues:
            validation_results['safety_issues'].append(f"Parameter drift detected: {', '.join(param_drift_issues)}")
        # Check if there are frequent rapid adjustments (could indicate instability)
        if hasattr(self, '_prev_validation_curvatures'):
            if len(self._prev_validation_curvatures) >= 5:
                recent_changes = [abs(self._prev_validation_curvatures[i] - self._prev_validation_curvatures[i-1])
                                  for i in range(1, len(self._prev_validation_curvatures))]
                avg_change = np.mean(recent_changes)
                if avg_change > 0.05:  # Large changes in consecutive validations
                    validation_results['safety_issues'].append(f"Rapid curvature changes detected: {avg_change:.4f}")
                # Critical check for very rapid changes
                if avg_change > 0.1:  # Very large changes in consecutive validations
                    validation_results['critical_violations'].append(f"CRITICAL: Very rapid curvature changes detected: {avg_change:.4f}")
                    validation_results['is_safe'] = False
        else:
            self._prev_validation_curvatures = []
        self._prev_validation_curvatures.append(desired_curvature)
        self._prev_validation_curvatures = self._prev_validation_curvatures[-20:]  # Keep more values for better trend analysis
        # Calculate safety confidence based on various factors with more weight on critical issues
        confidence_factors = []
        # Parameter stability contributes to confidence (lower drift = higher confidence)
        total_param_drift = sum(abs(self.adaptive_params[p] - 1.0) if p not in ['curvature_bias']
                                else abs(self.adaptive_params[p]) * 5.0  # Give more weight to curvature bias # for stability calculation
                                for p in ['lateral_control_factor', 'acceleration_factor', 'curvature_bias'])
        drift_confidence = max(0.05, 1.0 - min(1.0, total_param_drift))  # More conservative
        confidence_factors.append(drift_confidence)
        # Model accuracy contributes to confidence (lower error = higher confidence)
        if len(self.model_accuracy_history) > 0:
            recent_errors = [rec['error'] for rec in list(self.model_accuracy_history)[-10:]]
            if recent_errors:
                avg_error = np.mean(recent_errors)
                accuracy_confidence = max(0.05, 1.0 - min(1.0, avg_error * 20))  # More sensitive to errors
                confidence_factors.append(accuracy_confidence)
        # Vehicle state contributes to confidence (more stable state = higher confidence)
        state_confidence = max(0.3, min(1.0, v_ego / 20.0)) if v_ego < 20 else 0.8  # More conservative at high speeds
        confidence_factors.append(state_confidence)
        validation_results['confidence_in_safety'] = np.mean(confidence_factors) if confidence_factors else 1.0
        # Further reduce confidence if we had to apply overrides or detected critical issues
        if validation_results['overrides_applied'] or validation_results['critical_violations']:
            validation_results['confidence_in_safety'] *= 0.5  # Halve confidence if critical issues found
        # Add results from enhanced safety validation if available
        if enhanced_validation:
            if not enhanced_validation['is_safe']:
                validation_results['is_safe'] = False
                validation_results['safety_issues'].extend(enhanced_validation['safety_issues'])
                validation_results['confidence_in_safety'] *= 0.7  # Reduce confidence more significantly
                if enhanced_validation.get('critical_violations'):
                    validation_results['critical_violations'].extend(enhanced_validation['critical_violations'])
        # Log safety validation results with critical emphasis
        if validation_results['critical_violations']:
            cloudlog.error(f"CRITICAL SELF-LEARNING SAFETY VIOLATIONS - Safe: {validation_results['is_safe']}, "
                          f"Issues: {validation_results['critical_violations']}, "
                          f"Curvature: {desired_curvature:.4f} -> {validation_results['corrected_curvature']:.4f}, "
                          f"Acceleration: {desired_acceleration:.2f} -> {validation_results['corrected_acceleration']:.2f}")
        elif not validation_results['is_safe'] or validation_results['confidence_in_safety'] < 0.7:
            cloudlog.warning(f"Self-Learning Safety Validation - Safe: {validation_results['is_safe']}, "
                           f"Confidence: {validation_results['confidence_in_safety']:.2f}, "
                           f"Issues: {validation_results['safety_issues']}, "
                           f"Curvature: {desired_curvature:.4f} -> {validation_results['corrected_curvature']:.4f}, "
                           f"Acceleration: {desired_acceleration:.2f} -> {validation_results['corrected_acceleration']:.2f}")
        # Calculate average recent change if available
        avg_recent_change = 0
        if hasattr(self, '_prev_validation_curvatures') and len(self._prev_validation_curvatures) >= 5:
            recent_changes = [abs(self._prev_validation_curvatures[i] - self._prev_validation_curvatures[i-1])
                              for i in range(1, len(self._prev_validation_curvatures))]
            if recent_changes:
                avg_recent_change = np.mean(recent_changes)
        # Set validation passed flag
        validation_results['validation_passed'] = len(validation_results['critical_violations']) == 0
        validation_results['validation_details'] = {
            'v_ego': v_ego,
            'lateral_accel': lateral_accel,
            'param_drift_count': len(param_drift_issues),
            'critical_drift_count': len(critical_drift_issues),
            'avg_recent_change': avg_recent_change,
            'validation_time': enhanced_validation['validation_time'] if enhanced_validation else 0,
            'learning_context': self.learning_context.copy()
        }
        return validation_results
    def check_for_reset_request(self):
        """
        Check for reset request from parameters and perform reset if requested.
        This provides a user-facing mechanism to reset learned parameters.
        """
        try {
            # Check if reset parameter has been set
            reset_request = self.params.get("ResetSelfLearning")
            if reset_request and reset_request.lower() == "1") {
                cloudlog.info("Reset request detected, resetting self-learning state")
                self.reset_learning_state()
                # Clear the reset parameter so it doesn't trigger again
                self.params.delete("ResetSelfLearning")
            }
        } catch (Exception e) {
            cloudlog.warning(f"Error checking for reset request: {e}")
        }
