#!/usr/bin/env python3
"""
Safety Measures for Self-Learning Autonomous Driving System

This module implements safety checks and validation for the self-learning system
to ensure that learning-based adjustments do not compromise safety.
"""

import numpy as np
from typing import Dict, List, Tuple
from openpilot.common.swaglog import cloudlog
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log, car


class SelfLearningSafety:
    """
    Implements safety measures for the self-learning system.
    
    Ensures that learning-based adjustments remain within safe bounds
    and do not introduce unsafe driving behavior.
    """
    
    def __init__(self):
        # Safety thresholds
        self.max_lateral_acceleration = 3.0  # m/s^2
        self.max_lateral_jerk = 5.0  # m/s^3
        self.max_curvature = 0.5  # 1/m (approximately 2m turning radius)
        self.min_curvature = -0.5  # 1/m (approximately 2m turning radius)
        
        # Learning safety limits
        self.max_param_adjustment_rate = 0.1  # Maximum rate of parameter change
        self.min_adaptive_factor = 0.5  # Minimum allowed adaptive factor
        self.max_adaptive_factor = 2.0  # Maximum allowed adaptive factor
        
        # Safety monitoring filters
        self.curvature_rate_filter = FirstOrderFilter(0.0, 1.0, 0.05)  # 20Hz cutoff
        self.acceleration_filter = FirstOrderFilter(0.0, 1.0, 0.05)  # 20Hz cutoff
        
        # Safety state tracking
        self.prev_curvature = 0.0
        self.prev_adjustment_time = 0.0
        self.safety_violations = 0
        self.learning_safety_score = 1.0  # 1.0 = fully safe, 0.0 = unsafe
        self.learning_frozen = False  # Flag to freeze learning when unsafe
        
        # Critical safety limits that should never be exceeded
        self.critical_curvature_limit = 0.8  # Very high curvature threshold
        self.critical_acceleration_limit = 5.0  # Very high acceleration threshold
        
        cloudlog.info("Self-Learning Safety system initialized")
    
    def validate_curvature_adjustment(self, original_curvature: float, adjusted_curvature: float, 
                                    v_ego: float) -> Tuple[float, bool]:
        """
        Validate curvature adjustment for safety compliance.
        
        Args:
            original_curvature: Original curvature from model
            adjusted_curvature: Adjusted curvature from learning system
            v_ego: Vehicle speed
            
        Returns:
            Tuple of (safe_curvature, is_safe)
        """
        # Calculate maximum safe curvature based on speed
        max_safe_curvature = self._calculate_max_safe_curvature(v_ego)
        
        # Apply curvature limits
        clamped_curvature = np.clip(adjusted_curvature, 
                                   self.min_curvature, 
                                   self.max_curvature)
        
        # Also limit based on speed-dependent safety
        clamped_curvature = np.clip(clamped_curvature,
                                   -max_safe_curvature,
                                   max_safe_curvature)
        
        # Check for excessive curvature changes (jerk limits)
        current_time = self._get_current_time()
        time_diff = current_time - self.prev_adjustment_time
        
        if time_diff > 0:
            curvature_rate = abs(clamped_curvature - self.prev_curvature) / time_diff
            
            # Apply jerk-based limiting
            max_curvature_rate = self._calculate_max_curvature_rate(v_ego)
            if curvature_rate > max_curvature_rate:
                # Limit the change to stay within jerk limits
                max_curvature_change = max_curvature_rate * time_diff
                curvature_diff = clamped_curvature - self.prev_curvature
                clamped_curvature = self.prev_curvature + np.clip(
                    curvature_diff, -max_curvature_change, max_curvature_change
                )
        
        self.prev_curvature = clamped_curvature
        self.prev_adjustment_time = current_time
        
        # Determine if adjustment is safe
        is_safe = (abs(clamped_curvature) <= max_safe_curvature and 
                  abs(clamped_curvature - original_curvature) / max(abs(original_curvature), 0.01) <= 2.0)  # Max 2x change
        
        return clamped_curvature, is_safe
    
    def validate_acceleration_adjustment(self, original_accel: float, adjusted_accel: float) -> Tuple[float, bool]:
        """
        Validate acceleration adjustment for safety compliance.
        
        Args:
            original_accel: Original acceleration from model
            adjusted_accel: Adjusted acceleration from learning system
            
        Returns:
            Tuple of (safe_acceleration, is_safe)
        """
        # Apply acceleration limits
        clamped_accel = np.clip(adjusted_accel, -self.max_lateral_acceleration, self.max_lateral_acceleration)
        
        # Check for excessive acceleration changes (jerk limits)
        filtered_accel = self.acceleration_filter.update(clamped_accel)
        accel_change = abs(clamped_accel - original_accel)
        
        # Check if the adjustment is reasonable (not more than 100% change)
        is_safe = True
        if abs(original_accel) > 0.1:  # Only check if original acceleration is significant
            change_ratio = accel_change / abs(original_accel)
            if change_ratio > 2.0:  # More than 200% change
                clamped_accel = original_accel * np.sign(clamped_accel) * min(2.0, abs(clamped_accel/original_accel))
                is_safe = False
        
        # Critical safety check - never exceed critical limits
        clamped_accel = np.clip(clamped_accel, -self.critical_acceleration_limit, self.critical_acceleration_limit)
        
        return clamped_accel, is_safe
    
    def validate_parameter_adjustment(self, param_name: str, current_value: float, 
                                    proposed_value: float, v_ego: float) -> Tuple[float, bool]:
        """
        Validate adaptive parameter adjustment for safety compliance.
        
        Args:
            param_name: Name of the parameter being adjusted
            current_value: Current parameter value
            proposed_value: Proposed new parameter value
            v_ego: Vehicle speed
            
        Returns:
            Tuple of (safe_parameter_value, is_safe)
        """
        is_safe = True
        
        if param_name == 'lateral_control_factor':
            # Limit the rate of change for lateral control factor
            max_change_rate = self.max_param_adjustment_rate
            time_factor = min(1.0, v_ego / 20.0)  # More conservative at high speeds
            max_change = max_change_rate * time_factor
            
            change = proposed_value - current_value
            clamped_change = np.clip(change, -max_change, max_change)
            clamped_value = current_value + clamped_change
            
            # Apply absolute bounds
            clamped_value = np.clip(clamped_value, self.min_adaptive_factor, self.max_adaptive_factor)
            
            is_safe = (abs(clamped_change) <= max_change and 
                      self.min_adaptive_factor <= clamped_value <= self.max_adaptive_factor)
        
        elif param_name == 'curvature_bias':
            # Limit curvature bias based on speed and road conditions
            max_bias_speed_factor = min(0.02, max(0.001, v_ego * 0.0005))
            clamped_value = np.clip(proposed_value, -max_bias_speed_factor, max_bias_speed_factor)
            
            is_safe = abs(clamped_value) <= max_bias_speed_factor
        
        elif param_name == 'acceleration_factor':
            # Similar constraints as lateral control factor
            max_change_rate = self.max_param_adjustment_rate
            change = proposed_value - current_value
            clamped_change = np.clip(change, -max_change_rate, max_change_rate)
            clamped_value = current_value + clamped_change
            clamped_value = np.clip(clamped_value, self.min_adaptive_factor, self.max_adaptive_factor)
            
            is_safe = (abs(clamped_change) <= max_change_rate and
                      self.min_adaptive_factor <= clamped_value <= self.max_adaptive_factor)
        
        else:
            # Default behavior for other parameters
            clamped_value = proposed_value
        
        return clamped_value, is_safe
    
    def update_safety_score(self, CS: car.CarState, model_outputs: dict,
                           adjusted_outputs: dict) -> float:
        """
        Update the learning safety score based on current driving conditions.

        Args:
            CS: Current car state
            model_outputs: Original model outputs
            adjusted_outputs: Outputs after learning adjustments

        Returns:
            Updated safety score (0.0 to 1.0)
        """
        scores = []

        # Evaluate curvature adjustments based on absolute values against physical limits
        if 'desired_curvature' in model_outputs and 'desired_curvature' in adjusted_outputs:
            orig_curv = model_outputs['desired_curvature']
            adj_curv = adjusted_outputs['desired_curvature']

            # Check if adjusted curvature is within safe physical limits
            max_safe_curvature = self._calculate_max_safe_curvature(CS.vEgo)
            adj_curv_safe_ratio = min(1.0, abs(adj_curv) / max_safe_curvature) if max_safe_curvature > 0 else 0.0
            # Higher score for safer adjustments (closer to 1.0)
            curvature_score = max(0.0, 1.0 - adj_curv_safe_ratio)
            scores.append(curvature_score)

            # Also check the change magnitude for stability
            abs_change = abs(adj_curv - orig_curv)
            if abs_change > 0.1:  # Large change threshold
                # If change is large, reduce score proportionally
                change_score = max(0.0, 1.0 - (abs_change / 0.2))  # 0.2 as max expected change
                scores.append(change_score)

        # Evaluate acceleration adjustments based on absolute values
        if 'desired_acceleration' in model_outputs and 'desired_acceleration' in adjusted_outputs:
            orig_accel = model_outputs['desired_acceleration']
            adj_accel = adjusted_outputs['desired_acceleration']

            # Check if adjusted acceleration is within safe limits
            adj_accel_safe_ratio = min(1.0, abs(adj_accel) / self.max_lateral_acceleration)
            accel_score = max(0.0, 1.0 - adj_accel_safe_ratio)
            scores.append(accel_score)

            # Also check the magnitude of change for stability
            abs_accel_change = abs(adj_accel - orig_accel)
            if abs_accel_change > 0.5:  # Large acceleration change threshold
                accel_change_score = max(0.0, 1.0 - (abs_accel_change / 1.0))  # 1.0 as max expected change
                scores.append(accel_change_score)

        # Evaluate based on vehicle state
        if CS.vEgo > 30:  # High speed - be more conservative
            speed_score = max(0.3, 1.0 - (CS.vEgo - 30) * 0.01)  # Decreases as speed increases beyond 30 m/s
            scores.append(speed_score)
        else:
            speed_score = 1.0
            scores.append(speed_score)

        # Evaluate system stability - check for rapid parameter changes
        stability_score = 1.0  # Default high stability
        if hasattr(self, 'prev_adj_curvature') and 'desired_curvature' in adjusted_outputs:
            current_curv = adjusted_outputs['desired_curvature']
            change_from_prev = abs(current_curv - self.prev_adj_curvature)
            if change_from_prev > 0.05:  # Threshold for stable behavior
                stability_score = max(0.0, 1.0 - (change_from_prev / 0.1))  # 0.1 as max expected stability change
                scores.append(stability_score)

        if 'desired_curvature' in adjusted_outputs:
            self.prev_adj_curvature = adjusted_outputs['desired_curvature']

        # Calculate overall safety score
        if scores:
            overall_score = min(scores)  # Use minimum score as conservative measure
        else:
            overall_score = 1.0

        # Apply time-based smoothing to prevent rapid fluctuations
        if hasattr(self, 'prev_safety_score'):
            # Exponential moving average for smoothing
            alpha = 0.1  # Smoothing factor
            self.learning_safety_score = alpha * overall_score + (1 - alpha) * self.prev_safety_score
        else:
            self.learning_safety_score = overall_score

        self.prev_safety_score = self.learning_safety_score

        return self.learning_safety_score
    
    def should_freeze_learning(self, CS: car.CarState, safety_score: float) -> bool:
        """
        Determine if learning should be frozen based on safety conditions.
        
        Args:
            CS: Current car state
            safety_score: Current safety score
            
        Returns:
            True if learning should be frozen
        """
        # Freeze learning if safety score is too low
        if safety_score < 0.3:
            cloudlog.warning(f"Safety score too low ({safety_score:.2f}), freezing learning")
            return True
        
        # Freeze learning during emergency situations
        if CS.brakePressed and CS.vEgo > 5.0:
            # Emergency braking
            cloudlog.info("Emergency braking detected, temporarily freezing learning")
            return True
        
        # Freeze learning if steering angle is at limits
        if abs(CS.steeringAngleDeg) > 70:  #接近极限
            cloudlog.info("Steering near limits, freezing learning")
            return True
        
        # Check for unsafe lateral acceleration
        if CS.vEgo > 5:  # Only check when moving
            lateral_accel = CS.vEgo * CS.vEgo * self.prev_curvature
            if abs(lateral_accel) > self.max_lateral_acceleration * 0.9:  # 90% of limit
                cloudlog.warning(f"High lateral acceleration ({lateral_accel:.2f} m/s²), freezing learning")
                return True
        
        return False
    
    def _calculate_max_safe_curvature(self, v_ego: float) -> float:
        """
        Calculate maximum safe curvature based on vehicle speed and physical limits.
        
        Args:
            v_ego: Vehicle speed in m/s
            
        Returns:
            Maximum safe curvature in 1/m
        """
        if v_ego < 0.1:  # Very low speed
            return self.max_curvature
            
        # Based on max lateral acceleration = v^2 * curvature
        # So max_curvature = max_lat_accel / v^2
        max_curvature = self.max_lateral_acceleration / (v_ego * v_ego)
        
        # Also limit based on vehicle capabilities
        return min(max_curvature, self.max_curvature)
    
    def _calculate_max_curvature_rate(self, v_ego: float) -> float:
        """
        Calculate maximum safe curvature rate (jerk) based on speed.
        
        Args:
            v_ego: Vehicle speed in m/s
            
        Returns:
            Maximum safe curvature rate in 1/(m*s)
        """
        # Higher speeds require more conservative curvature changes
        base_rate = self.max_lateral_jerk / (v_ego * v_ego + 1.0)  # Add 1.0 to prevent division by zero at standstill
        return min(base_rate, 0.5)  # Cap at reasonable maximum
    
    def _get_current_time(self) -> float:
        """
        Get current time for timing calculations.
        """
        import time
        return time.time()
    
    def get_safety_recommendation(self, CS: car.CarState, model_output: dict) -> log.ControlsState.SafetyRecommendation:
        """
        Provide safety recommendations based on current conditions and learning state.
        
        Args:
            CS: Current car state
            model_output: Current model output
            
        Returns:
            Safety recommendation enum
        """
        # Default to normal operation
        safety_recommendation = log.ControlsState.SafetyRecommendation.normal
        
        # Check various safety conditions
        if self.learning_safety_score < 0.5:
            safety_recommendation = log.ControlsState.SafetyRecommendation.reduced_engagement
        elif self.learning_safety_score < 0.2:
            safety_recommendation = log.ControlsState.SafetyRecommendation.prepare_for_disengage
        elif self.safety_violations > 5:
            safety_recommendation = log.ControlsState.SafetyRecommendation.prepare_for_disengage
        
        # Check for physical limit violations
        if hasattr(CS, 'steeringTorque'):
            if abs(CS.steeringTorque) > 1800:  # Assuming typical torque limits
                safety_recommendation = log.ControlsState.SafetyRecommendation.prepare_for_disengage
        
        if abs(CS.steeringAngleDeg) > 80:
            safety_recommendation = log.ControlsState.SafetyRecommendation.prepare_for_disengage
        
        return safety_recommendation


class SafeSelfLearningManager:
    """
    Wrapper that combines self-learning and safety functions.
    """
    
    def __init__(self, CP, CP_SP):
        self.learning_manager = SelfLearningManager(CP, CP_SP)
        self.safety = SelfLearningSafety()
        self.enabled = True
    
    def update(self, CS, desired_curvature, actual_curvature, steering_torque, v_ego,
               model_confidence=1.0, model_prediction_error=None):
        """
        Update the safe self-learning system.

        Args:
            CS: Current car state
            desired_curvature: Model's desired curvature
            actual_curvature: Actual vehicle curvature
            steering_torque: Current steering torque
            v_ego: Vehicle speed
            model_confidence: Model confidence score
            model_prediction_error: Difference between model output and actual vehicle behavior
        """
        if not self.enabled:
            return

        # Update safety monitoring
        self.learning_manager.update_from_model_accuracy(
            desired_curvature, actual_curvature, v_ego, model_confidence
        )

        # Update from driver interventions - pass model prediction error for context-aware learning
        self.learning_manager.update_from_driver_intervention(
            CS, desired_curvature, actual_curvature, steering_torque, v_ego, model_prediction_error
        )

        # Periodic updates
        self.learning_manager.periodic_update()

        # Check if learning should be frozen for safety
        model_outputs = {'desired_curvature': desired_curvature}
        # Get the adjusted output from the learning manager to use for safety scoring
        adjusted_curvature = self.learning_manager.adjust_curvature_prediction(desired_curvature, v_ego)
        adjusted_outputs = {'desired_curvature': adjusted_curvature}  # FIXED: Use adjusted output, not original

        safety_score = self.safety.update_safety_score(CS, model_outputs, adjusted_outputs)
        freeze_learning = self.safety.should_freeze_learning(CS, safety_score)

        if freeze_learning:
            # For now, just log the event - in production, might want to implement actual freezing
            cloudlog.warning("Learning would be frozen based on safety conditions")
    
    def adjust_curvature(self, original_curvature: float, v_ego: float) -> float:
        """
        Safely adjust curvature with validation.
        
        Args:
            original_curvature: Original curvature from model
            v_ego: Vehicle speed
            
        Returns:
            Safely adjusted curvature
        """
        # Apply learning adjustment
        adjusted_curvature = self.learning_manager.adjust_curvature_prediction(
            original_curvature, v_ego
        )
        
        # Validate for safety
        safe_curvature, is_safe = self.safety.validate_curvature_adjustment(
            original_curvature, adjusted_curvature, v_ego
        )
        
        if not is_safe:
            # Revert to original if adjustment is unsafe
            cloudlog.warning(f"Curvature adjustment unsafe, using original: {original_curvature:.5f} vs {adjusted_curvature:.5f}")
            return original_curvature
        
        return safe_curvature
    
    def adjust_acceleration(self, original_accel: float, v_ego: float) -> float:
        """
        Safely adjust acceleration with validation.
        
        Args:
            original_accel: Original acceleration from model
            v_ego: Vehicle speed
            
        Returns:
            Safely adjusted acceleration
        """
        # Apply learning adjustment
        adjusted_accel = self.learning_manager.adjust_acceleration_prediction(
            original_accel, v_ego
        )
        
        # Validate for safety
        safe_accel, is_safe = self.safety.validate_acceleration_adjustment(
            original_accel, adjusted_accel
        )
        
        if not is_safe:
            # Revert to original if adjustment is unsafe
            cloudlog.warning(f"Acceleration adjustment unsafe, using original: {original_accel:.3f} vs {adjusted_accel:.3f}")
            return original_accel
        
        return safe_accel