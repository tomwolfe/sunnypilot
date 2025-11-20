#!/usr/bin/env python3
"""
Backup safety systems for sunnypilot
Provides redundant safety checks and backup control paths for critical functions
"""
import numpy as np
from typing import Tuple, Optional
from openpilot.common.swaglog import cloudlog


class SafetyBackupSystem:
    """
    Backup safety system that provides redundant checks for critical functions
    """

    def __init__(self):
        self.last_safe_curvature = 0.0
        self.last_safe_acceleration = 0.0
        self.safety_engaged = False
        self.fallback_active = False

    def validate_curvature(self, desired_curvature: float, v_ego: float, 
                          max_lateral_accel: float = 3.0) -> float:
        """
        Validate curvature against physical limits and vehicle dynamics
        :param desired_curvature: Desired curvature from primary system
        :param v_ego: Current vehicle speed
        :param max_lateral_accel: Maximum allowed lateral acceleration
        :return: Validated curvature that respects safety limits
        """
        try:
            # Calculate maximum safe curvature based on speed and max lateral acceleration
            if v_ego > 0.1:  # Only validate at meaningful speeds
                max_curvature = max_lateral_accel / (v_ego ** 2)
                validated_curvature = np.clip(desired_curvature, -max_curvature, max_curvature)
            else:
                # At very low speeds, allow higher curvature but cap it reasonably
                validated_curvature = np.clip(desired_curvature, -0.2, 0.2)
            
            # Store as last safe value
            self.last_safe_curvature = validated_curvature
            
            # Log if the desired curvature was out of bounds
            if abs(desired_curvature) > abs(validated_curvature) * 1.01:  # Small tolerance for floating point
                cloudlog.warning(f"Curvature validation: {desired_curvature} limited to {validated_curvature}")
            
            return validated_curvature
        except Exception as e:
            cloudlog.error(f"Error in curvature validation: {e}")
            # Return safe fallback
            return 0.0  # Return neutral (straight) curvature as safe fallback

    def validate_acceleration(self, desired_accel: float, 
                            max_accel: float = 2.0, max_decel: float = -3.5) -> float:
        """
        Validate longitudinal acceleration against safe limits
        :param desired_accel: Desired acceleration from primary system
        :param max_accel: Maximum allowed positive acceleration
        :param max_decel: Maximum allowed negative acceleration (deceleration)
        :return: Validated acceleration that respects safety limits
        """
        try:
            validated_accel = np.clip(desired_accel, max_decel, max_accel)
            
            # Store as last safe value
            self.last_safe_acceleration = validated_accel
            
            # Log if the desired acceleration was out of bounds
            if (desired_accel > validated_accel or desired_accel < validated_accel):
                cloudlog.warning(f"Acceleration validation: {desired_accel} limited to {validated_accel}")
            
            return validated_accel
        except Exception as e:
            cloudlog.error(f"Error in acceleration validation: {e}")
            # Return safe fallback
            return 0.0  # Return neutral (no acceleration) as safe fallback

    def validate_environmental_risk(self, risk_score: float) -> float:
        """
        Validate environmental risk score to ensure it's in valid range
        :param risk_score: Risk score from primary system
        :return: Validated risk score between 0.0 and 1.0
        """
        try:
            validated_risk = np.clip(risk_score, 0.0, 1.0)
            
            # Log if the risk score was out of bounds
            if risk_score != validated_risk:
                cloudlog.warning(f"Risk score validation: {risk_score} limited to {validated_risk}")
            
            return validated_risk
        except Exception as e:
            cloudlog.error(f"Error in risk score validation: {e}")
            # Return safe fallback (high risk means conservative behavior)
            return 1.0  # Return maximum risk for safety

    def get_backup_curvature(self, v_ego: float) -> float:
        """
        Get a safe backup curvature if primary system fails
        :param v_ego: Current vehicle speed
        :return: Safe curvature value
        """
        # Return neutral (straight) curvature as primary backup
        # This is the safest option when primary system fails
        self.fallback_active = True
        cloudlog.warning("Backup system: Using neutral curvature due to primary system failure")
        return 0.0

    def get_backup_acceleration(self) -> float:
        """
        Get a safe backup acceleration if primary system fails
        :return: Safe acceleration value
        """
        # Return neutral (no acceleration) as primary backup
        # This is the safest option when primary system fails
        self.fallback_active = True
        cloudlog.warning("Backup system: Using neutral acceleration due to primary system failure")
        return 0.0

    def emergency_stop(self) -> Tuple[float, float]:
        """
        Activate emergency stop by returning safe values
        :return: Tuple of (safe_curvature, safe_acceleration) for emergency stop
        """
        self.safety_engaged = True
        self.fallback_active = True
        cloudlog.error("Backup system: Emergency stop activated")
        return (0.0, -3.5)  # Neutral steering, maximum safe deceleration

    def is_system_operational(self) -> bool:
        """
        Check if primary system is operational
        :return: True if primary system appears to be working correctly
        """
        return not self.fallback_active and not self.safety_engaged


class RedundantControlValidator:
    """
    Provides redundant validation for control outputs using independent calculation
    """
    
    def __init__(self):
        self.backup_system = SafetyBackupSystem()
        
    def validate_control_outputs(self, desired_curvature: float, desired_accel: float, 
                               v_ego: float, max_lateral_accel: float = 3.0,
                               max_accel: float = 2.0, max_decel: float = -3.5) -> Tuple[float, float]:
        """
        Validate control outputs using redundant safety checks
        :param desired_curvature: Desired curvature from primary system
        :param desired_accel: Desired acceleration from primary system
        :param v_ego: Current vehicle speed
        :param max_lateral_accel: Maximum allowed lateral acceleration
        :param max_accel: Maximum allowed positive acceleration
        :param max_decel: Maximum allowed negative acceleration (deceleration)
        :return: Tuple of (validated_curvature, validated_acceleration)
        """
        # Validate curvature
        validated_curvature = self.backup_system.validate_curvature(
            desired_curvature, v_ego, max_lateral_accel
        )
        
        # Validate acceleration
        validated_accel = self.backup_system.validate_acceleration(
            desired_accel, max_accel, max_decel
        )
        
        return (validated_curvature, validated_accel)
    
    def validate_and_backup(self, desired_curvature: float, desired_accel: float,
                          v_ego: float, model_confidence: float,
                          environmental_risk: float) -> Tuple[float, float]:
        """
        Perform full validation with backup systems if needed
        :param desired_curvature: Desired curvature from primary system
        :param desired_accel: Desired acceleration from primary system
        :param v_ego: Current vehicle speed
        :param model_confidence: Confidence in model outputs (0.0-1.0)
        :param environmental_risk: Environmental risk assessment (0.0-1.0)
        :return: Tuple of (safe_curvature, safe_acceleration)
        """
        # Check if model confidence is too low for safe operation
        if model_confidence < 0.5 or environmental_risk > 0.8:
            cloudlog.warning(f"Low confidence ({model_confidence}) or high risk ({environmental_risk}), using validated outputs")
        
        # Perform redundant validation using independent calculations
        validated_curvature, validated_accel = self.validate_control_outputs(
            desired_curvature, desired_accel, v_ego
        )
        
        # Additional cross-verification checks
        try:
            # Check for physically impossible values
            if v_ego > 0.1 and abs(validated_curvature) > 0.5:
                # At speed > 0.1 m/s, curvature > 0.5 is extremely high (would require impossibly sharp turn)
                cloudlog.warning(f"Unsafe curvature {validated_curvature} at speed {v_ego}, using backup")
                validated_curvature = self.backup_system.get_backup_curvature(v_ego)
            
            if abs(validated_accel) > 5.0:
                # Acceleration/deceleration > 5 m/s^2 is extremely high (unsafe)
                cloudlog.warning(f"Unsafe acceleration {validated_accel}, using backup")
                validated_accel = self.backup_system.get_backup_acceleration()
                
        except Exception as e:
            cloudlog.error(f"Error in cross-verification: {e}")
            # Use backup values if verification fails
            validated_curvature = self.backup_system.get_backup_curvature(v_ego)
            validated_accel = self.backup_system.get_backup_acceleration()
        
        return (validated_curvature, validated_accel)