#!/usr/bin/env python3
"""
Adaptive system behavior for sunnypilot based on driving conditions
Implements context-aware adjustments to control parameters
"""
import numpy as np
import math
from enum import Enum
from typing import Dict, Tuple, Optional
from cereal import log
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import get_safe_speed_from_curvature, adjust_curvature_for_road_conditions
from openpilot.selfdrive.controls.lib.lateral_safety import adjust_lateral_limits_for_conditions


class DrivingPersonality(Enum):
    """Different driving personalities based on conditions and user preference"""
    CONSERVATIVE = "conservative"
    BALANCED = "balanced" 
    AGGRESSIVE = "aggressive"
    ADAPTIVE = "adaptive"


class AdaptiveController:
    """
    Main controller for adaptive behavior based on driving conditions
    """
    
    def __init__(self):
        self.personality = DrivingPersonality.ADAPTIVE
        self.current_road_curvature = 0.0
        self.current_speed = 0.0
        self.environmental_risk = 0.0
        
        # Adaptive parameters
        self.lateral_accel_limit = 3.0  # Base lateral acceleration limit (m/s^2)
        self.longitudinal_accel_limit = 2.0  # Base longitudinal acceleration limit (m/s^2)
        self.steer_ratio_factor = 1.0  # Factor to adjust steering response
        self.curvature_rate_limit = 0.1  # Maximum rate of curvature change
        
        # Condition tracking
        self.is_curving = False
        self.is_on_grade = False
        self.model_confidence_low = False
        self.visibility_poor = False
        
    def update_conditions(self, v_ego: float, curvature: float, model_v2, 
                         road_pitch: float = 0.0, model_confidence: float = 1.0, 
                         visibility_factor: float = 1.0):
        """
        Update current driving conditions
        :param v_ego: Current vehicle speed
        :param curvature: Current curvature
        :param model_v2: Model output message
        :param road_pitch: Road grade/pitch
        :param model_confidence: Model confidence (0.0-1.0)
        :param visibility_factor: Visibility quality (0.0-1.0)
        """
        self.current_speed = v_ego
        self.current_road_curvature = curvature
        self.environmental_risk = 1.0 - model_confidence
        
        # Update condition flags
        self.is_curving = abs(curvature) > 0.002  # Detect if we're in a curve
        self.is_on_grade = abs(road_pitch) > 0.02  # Detect if on grade > 2%
        self.model_confidence_low = model_confidence < 0.6
        self.visibility_poor = visibility_factor < 0.5
        
    def get_adaptive_lateral_limits(self) -> Tuple[float, float]:
        """
        Get adaptive lateral acceleration limits based on current conditions
        :return: Tuple of (min_lat_accel, max_lat_accel)
        """
        base_limit = self.lateral_accel_limit
        
        # Adjust based on current conditions
        adjustment = 1.0
        
        if self.is_curving:
            adjustment *= 0.85  # More conservative in curves
        if self.is_on_grade:
            adjustment *= 0.9   # More conservative on grades
        if self.model_confidence_low:
            adjustment *= 0.75  # More conservative with low model confidence
        if self.visibility_poor:
            adjustment *= 0.8   # More conservative with poor visibility
        if self.current_speed > 25.0:  # ~55 mph
            adjustment *= 0.95  # More conservative at high speeds
            
        # Apply personality-based adjustment
        if self.personality == DrivingPersonality.CONSERVATIVE:
            adjustment *= 0.8
        elif self.personality == DrivingPersonality.AGGRESSIVE and not any([
            self.is_curving, self.model_confidence_low, self.visibility_poor
        ]):
            adjustment *= 1.1  # Slightly more aggressive when conditions allow
        elif self.personality == DrivingPersonality.ADAPTIVE:
            # For adaptive mode, use calculated adjustments
            pass
            
        adjusted_limit = base_limit * adjustment
        return -adjusted_limit, adjusted_limit
    
    def get_adaptive_longitudinal_limits(self) -> Tuple[float, float]:
        """
        Get adaptive longitudinal acceleration limits based on current conditions
        :return: Tuple of (min_long_accel, max_long_accel)
        """
        from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
        
        # Base limits
        min_accel = ACCEL_MIN
        max_accel = ACCEL_MAX
        
        # Adjust based on conditions
        adjustment = 1.0
        
        if self.is_curving:
            adjustment *= 0.7   # Reduce acceleration in curves
        if self.is_on_grade:
            # Different adjustment for grades - be more conservative when going downhill
            if self.current_road_curvature < 0:  # Going downhill
                adjustment *= 0.8
            else:  # Going uphill
                adjustment *= 0.9
        if self.model_confidence_low:
            adjustment *= 0.7   # Be much more conservative with low model confidence
        if self.visibility_poor:
            adjustment *= 0.75  # Be more conservative with poor visibility
        if self.current_speed > 30.0:  # ~65 mph
            adjustment *= 0.8   # Be more conservative at very high speeds
            
        # Apply personality-based adjustment
        if self.personality == DrivingPersonality.CONSERVATIVE:
            adjustment *= 0.85
        elif self.personality == DrivingPersonality.AGGRESSIVE and not any([
            self.is_curving, self.model_confidence_low, self.visibility_poor
        ]):
            adjustment *= 1.05  # Slightly more aggressive when conditions allow
            
        return min_accel * adjustment, max_accel * adjustment
    
    def get_adaptive_curvature_rate_limit(self) -> float:
        """
        Get adaptive curvature rate limit based on conditions
        :return: Maximum rate of curvature change
        """
        base_rate_limit = self.curvature_rate_limit
        
        # Adjust based on conditions
        adjustment = 1.0
        
        if self.is_curving:
            adjustment *= 0.7   # Slower curvature changes in curves
        if self.model_confidence_low:
            adjustment *= 0.6   # Much smoother changes with low confidence
        if self.visibility_poor:
            adjustment *= 0.7   # Smoother changes with poor visibility
        if self.current_speed > 20.0:  # ~45 mph
            adjustment *= 0.85  # Smoother changes at higher speeds
            
        return base_rate_limit * adjustment
    
    def adjust_curvature_for_conditions(self, desired_curvature: float, 
                                      prev_curvature: float, dt: float = 0.05) -> float:
        """
        Apply adaptive adjustments to desired curvature based on conditions
        :param desired_curvature: Raw desired curvature from model
        :param prev_curvature: Previous curvature for rate limiting
        :param dt: Time step
        :return: Adjusted curvature that respects adaptive limits
        """
        # Apply rate limiting based on current conditions
        max_curvature_change = self.get_adaptive_curvature_rate_limit() * dt
        
        # Rate limit the curvature change
        curvature_after_rate_limit = np.clip(
            desired_curvature,
            prev_curvature - max_curvature_change,
            prev_curvature + max_curvature_change
        )
        
        # Apply safety limits based on speed and curvature
        max_lat_accel, _ = self.get_adaptive_lateral_limits()
        safe_curvature = get_safe_speed_from_curvature(abs(curvature_after_rate_limit), max_lat_accel)
        
        if safe_curvature < self.current_speed * 0.9:  # Current speed too high for curvature
            # Adjust curvature to be safer
            required_curvature = max_lat_accel / (self.current_speed * 0.9) ** 2
            if abs(curvature_after_rate_limit) > required_curvature:
                # Reduce the absolute value of curvature to be safer
                sign = 1.0 if curvature_after_rate_limit >= 0 else -1.0
                curvature_after_rate_limit = sign * min(abs(curvature_after_rate_limit), required_curvature)
        
        return curvature_after_rate_limit
    
    def get_adaptive_following_distance(self, lead_distance: float = 50.0,
                                      lead_velocity: float = 0.0) -> float:
        """
        Get adaptive following distance based on conditions
        :param lead_distance: Current distance to lead vehicle
        :param lead_velocity: Lead vehicle velocity
        :return: Adaptive following distance
        """
        # Base safe distance calculation
        base_distance = max(30.0, self.current_speed * 1.5)  # 1.5 second follow time as base, minimum 30m
        
        # Adjust based on conditions
        adjustment = 1.0
        
        if self.is_curving:
            adjustment *= 1.5  # Increase distance in curves
        if self.is_on_grade:
            adjustment *= 1.2  # Increase distance on grades
        if self.model_confidence_low:
            adjustment *= 1.4  # Much more distance with low confidence
        if self.visibility_poor:
            adjustment *= 1.3  # More distance with poor visibility
        if self.current_speed > 25.0:  # ~55 mph
            adjustment *= 1.2  # More distance at high speeds
            
        # Apply personality-based adjustment
        if self.personality == DrivingPersonality.CONSERVATIVE:
            adjustment *= 1.4
        elif self.personality == DrivingPersonality.AGGRESSIVE and not any([
            self.is_curving, self.model_confidence_low, self.visibility_poor
        ]):
            adjustment *= 0.85  # Closer following when conditions allow
            
        return base_distance * adjustment

    def get_adaptive_personality_from_conditions(self) -> DrivingPersonality:
        """
        Dynamically determine the appropriate driving personality based on conditions
        :return: Appropriate DrivingPersonality enum
        """
        # Check for hazardous conditions that require conservative behavior
        if (self.model_confidence_low or self.visibility_poor or 
            abs(self.current_road_curvature) > 0.01 or  # Sharp curve
            self.current_speed > 30.0):  # Very high speed
            return DrivingPersonality.CONSERVATIVE
            
        # Check for clear conditions where more aggressive driving is OK
        if (not self.is_curving and not self.is_on_grade and 
            not self.model_confidence_low and not self.visibility_poor):
            # In clear, safe conditions
            return DrivingPersonality.AGGRESSIVE
            
        # Otherwise, use balanced driving
        return DrivingPersonality.BALANCED


class ConditionBasedParameterTuner:
    """
    Automatically tunes control parameters based on driving conditions
    """
    
    def __init__(self):
        self.adaptive_controller = AdaptiveController()
        
    def update_with_conditions(self, sm, CP) -> Dict[str, float]:
        """
        Update control parameters based on current conditions
        :param sm: SubMaster with current messages
        :param CP: CarParams
        :return: Dictionary of adjusted parameters
        """
        car_state = sm['carState']
        model_v2 = sm['modelV2']
        
        # Extract relevant data
        v_ego = car_state.vEgo
        curvature = -car_state.steeringAngleDeg * CV.DEG_TO_RAD / (CP.wheelbase * 1.0)  # Simplified curvature calculation
        road_pitch = 0.0
        if len(car_state.orientationNED) > 1:
            road_pitch = car_state.orientationNED[1]  # Pitch angle
            
        model_confidence = getattr(model_v2.meta, 'confidence', 1.0) if hasattr(model_v2, 'meta') else 1.0
        
        # Use camera exposure or other indicators for visibility
        visibility_factor = 1.0  # Placeholder - would use actual visibility indicator
        
        # Update adaptive controller with current conditions
        self.adaptive_controller.update_conditions(
            v_ego, curvature, model_v2, road_pitch, model_confidence, visibility_factor
        )
        
        # Dynamically adjust personality based on conditions
        if self.adaptive_controller.personality == DrivingPersonality.ADAPTIVE:
            self.adaptive_controller.personality = self.adaptive_controller.get_adaptive_personality_from_conditions()
        
        # Get adjusted parameters
        adjusted_params = {}
        
        # Get adaptive lateral limits
        min_lat, max_lat = self.adaptive_controller.get_adaptive_lateral_limits()
        adjusted_params['max_lateral_accel'] = max_lat
        
        # Get adaptive longitudinal limits
        min_long, max_long = self.adaptive_controller.get_adaptive_longitudinal_limits()
        adjusted_params['max_long_accel'] = max_long
        adjusted_params['max_decel'] = min_long  # Negative value for deceleration
        
        # Get adaptive curvature rate limit
        adjusted_params['curvature_rate_limit'] = self.adaptive_controller.get_adaptive_curvature_rate_limit()
        
        # Adjust time constants based on conditions
        if self.adaptive_controller.model_confidence_low or self.adaptive_controller.visibility_poor:
            # Use longer planning horizons when less confident
            adjusted_params['long_plan_t'] = 6.0  # Longer planning time
            adjusted_params['lat_plan_t'] = 4.0   # Longer lateral planning time
        else:
            # Use normal planning times
            adjusted_params['long_plan_t'] = 5.0
            adjusted_params['lat_plan_t'] = 3.5
        
        # Adjust smoothing factors based on conditions
        if self.adaptive_controller.model_confidence_low:
            # Heavier smoothing when less confident
            adjusted_params['accel_smoothing_factor'] = 0.7
            adjusted_params['curvature_smoothing_factor'] = 0.6
        else:
            # Normal smoothing
            adjusted_params['accel_smoothing_factor'] = 0.9
            adjusted_params['curvature_smoothing_factor'] = 0.85
            
        return adjusted_params


class AdaptiveBehaviorManager:
    """
    Main manager for adaptive behavior that coordinates all adaptive systems
    """
    
    def __init__(self):
        self.condition_tuner = ConditionBasedParameterTuner()
        self.active_adjustments = {}
        
    def update(self, sm, CP) -> Dict[str, float]:
        """
        Update adaptive behavior based on current conditions
        :param sm: SubMaster with current messages
        :param CP: CarParams
        :return: Dictionary of all adaptive adjustments
        """
        # Get updated parameters based on conditions
        self.active_adjustments = self.condition_tuner.update_with_conditions(sm, CP)
        
        return self.active_adjustments
    
    def apply_adjustments(self, original_action, sm, CP):
        """
        Apply adaptive adjustments to the desired action from model
        :param original_action: Original action from neural network
        :param sm: SubMaster with current messages
        :param CP: CarParams
        :return: Adjusted action considering current conditions
        """
        try:
            # Update adaptive parameters
            adjustments = self.update(sm, CP)

            # Apply curvature adjustments
            adjusted_curvature = self.condition_tuner.adaptive_controller.adjust_curvature_for_conditions(
                original_action.desiredCurvature,
                sm['controlsState'].curvature if 'controlsState' in sm else 0.0
            )

            # Apply acceleration adjustments based on conditions
            desired_accel = original_action.desiredAcceleration
            max_accel = adjustments.get('max_long_accel', desired_accel)
            min_accel = adjustments.get('max_decel', -3.0)

            # Limit acceleration based on adaptive limits
            adjusted_accel = np.clip(desired_accel, min_accel, max_accel)

            # Create new action with adjustments
            adjusted_action = log.ModelDataV2.Action(
                desiredCurvature=float(adjusted_curvature),
                desiredAcceleration=float(adjusted_accel),
                shouldStop=original_action.shouldStop
            )

            return adjusted_action
        except Exception as e:
            # Log the error and re-raise for proper handling by controlling function
            cloudlog.error(f"AdaptiveBehaviorManager apply_adjustments error: {e}")
            raise
    
    def get_following_distance_adjustment(self, lead_distance, lead_velocity, sm):
        """
        Get adaptive following distance adjustment
        """
        current_adj = self.condition_tuner.adaptive_controller
        return current_adj.get_adaptive_following_distance(lead_distance, lead_velocity)