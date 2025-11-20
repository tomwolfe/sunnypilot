#!/usr/bin/env python3
"""
Backup safety systems for sunnypilot
Provides redundant safety checks and backup control paths for critical functions
"""
import numpy as np
import time
from typing import Tuple, Optional, Dict, Any
from openpilot.common.swaglog import cloudlog
from .autonomous_params import SAFETY_PARAMETERS


class SafetyBackupSystem:
    """
    Backup safety system that provides redundant checks for critical functions
    """

    def __init__(self):
        self.last_safe_curvature = 0.0
        self.last_safe_acceleration = 0.0
        self.last_validation_time = time.time()
        self.safety_engaged = False
        self.fallback_active = False
        self.validation_history = []  # Track validation events
        self.max_history = 100  # Max validation events to track

        # Use centralized safety parameters to ensure consistency
        self.DEFAULT_MAX_LATERAL_ACCEL = SAFETY_PARAMETERS['MAX_LATERAL_ACCEL']

    def validate_curvature(self, desired_curvature: float, v_ego: float,
                          max_lateral_accel: float = None,
                          vehicle_params: Optional[Dict[str, float]] = None) -> float:
        """
        Validate curvature against physical limits and vehicle dynamics
        :param desired_curvature: Desired curvature from primary system
        :param v_ego: Current vehicle speed
        :param max_lateral_accel: Maximum allowed lateral acceleration
        :param vehicle_params: Optional vehicle-specific parameters
        :return: Validated curvature that respects safety limits
        """
        try:
            # Use default max lateral acceleration from centralized parameters if not provided
            if max_lateral_accel is None:
                max_lateral_accel = self.DEFAULT_MAX_LATERAL_ACCEL

            # Get vehicle-specific parameters if available
            if vehicle_params and 'lateral_accel_limit' in vehicle_params:
                max_lateral_accel = vehicle_params['lateral_accel_limit']

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
                self._log_validation_event(
                    "curvature_clamped",
                    {"original": desired_curvature, "clamped": validated_curvature, "v_ego": v_ego}
                )
                cloudlog.warning(f"Curvature validation: {desired_curvature} limited to {validated_curvature}")

            return validated_curvature
        except Exception as e:
            cloudlog.error(f"Error in curvature validation: {e}")
            # Return safe fallback
            self._log_validation_event("curvature_error", {"error": str(e), "desired": desired_curvature})
            return 0.0  # Return neutral (straight) curvature as safe fallback

    def validate_acceleration(self, desired_accel: float,
                            max_accel: float = 2.0, max_decel: float = -3.5,
                            v_ego: float = 0.0,
                            vehicle_params: Optional[Dict[str, float]] = None) -> float:
        """
        Validate longitudinal acceleration against safe limits
        :param desired_accel: Desired acceleration from primary system
        :param max_accel: Maximum allowed positive acceleration
        :param max_decel: Maximum allowed negative acceleration (deceleration)
        :param v_ego: Current vehicle speed (used for speed-dependent limits)
        :param vehicle_params: Optional vehicle-specific parameters
        :return: Validated acceleration that respects safety limits
        """
        try:
            # Adjust limits based on vehicle parameters if available
            if vehicle_params:
                if 'max_accel_limit' in vehicle_params:
                    max_accel = vehicle_params['max_accel_limit']
                if 'max_decel_limit' in vehicle_params:
                    max_decel = vehicle_params['max_decel_limit']

            # For high speeds, possibly adjust acceleration limits for safety
            if v_ego > 25.0:  # Above ~90 km/h, be more conservative with acceleration
                max_accel = min(max_accel, 1.5)  # Limit acceleration at high speed

            validated_accel = np.clip(desired_accel, max_decel, max_accel)

            # Store as last safe value
            self.last_safe_acceleration = validated_accel

            # Log if the desired acceleration was out of bounds
            if (desired_accel > validated_accel or desired_accel < validated_accel):
                self._log_validation_event(
                    "acceleration_clamped",
                    {"original": desired_accel, "clamped": validated_accel, "v_ego": v_ego}
                )
                cloudlog.warning(f"Acceleration validation: {desired_accel} limited to {validated_accel}")

            return validated_accel
        except Exception as e:
            cloudlog.error(f"Error in acceleration validation: {e}")
            # Return safe fallback
            self._log_validation_event("acceleration_error", {"error": str(e), "desired": desired_accel})
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
                self._log_validation_event(
                    "risk_score_clamped",
                    {"original": risk_score, "clamped": validated_risk}
                )
                cloudlog.warning(f"Risk score validation: {risk_score} limited to {validated_risk}")

            return validated_risk
        except Exception as e:
            cloudlog.error(f"Error in risk score validation: {e}")
            self._log_validation_event("risk_score_error", {"error": str(e), "original": risk_score})
            # Return safe fallback (high risk means conservative behavior)
            return 1.0  # Return maximum risk for safety

    def validate_steering_rate(self, desired_curvature: float, current_curvature: float, dt: float = 0.1) -> float:
        """
        Validate that the requested curvature change rate is safe
        :param desired_curvature: Target curvature
        :param current_curvature: Current curvature
        :param dt: Time delta since last curvature command
        :return: Safe curvature that respects rate limits
        """
        try:
            # Calculate the rate of change of curvature (steering rate)
            if dt > 0:
                curvature_rate = abs(desired_curvature - current_curvature) / dt

                # Limit the rate of curvature change (max 0.5 1/m/s rate change)
                max_curvature_rate = 0.5
                if curvature_rate > max_curvature_rate:
                    # Calculate the safe curvature value based on rate limits
                    max_curvature_change = max_curvature_rate * dt
                    safe_curvature = current_curvature + np.sign(desired_curvature - current_curvature) * max_curvature_change
                    safe_curvature = np.clip(safe_curvature, current_curvature - max_curvature_change,
                                           current_curvature + max_curvature_change)

                    self._log_validation_event(
                        "steering_rate_limited",
                        {"original_rate": curvature_rate, "limited_rate": max_curvature_rate,
                         "original_curvature": desired_curvature, "safe_curvature": safe_curvature}
                    )
                    cloudlog.warning(f"Steering rate validation: Rate {curvature_rate} limited to {max_curvature_rate}, "
                                   f"Curvature {desired_curvature} limited to {safe_curvature}")
                    return safe_curvature

            return desired_curvature
        except Exception as e:
            cloudlog.error(f"Error in steering rate validation: {e}")
            self._log_validation_event("steering_rate_error", {"error": str(e), "desired": desired_curvature})
            return current_curvature  # Return current value for safety

    def get_backup_curvature(self, v_ego: float) -> float:
        """
        Get a safe backup curvature if primary system fails
        :param v_ego: Current vehicle speed
        :return: Safe curvature value
        """
        # Return neutral (straight) curvature as primary backup
        # This is the safest option when primary system fails
        self.fallback_active = True
        self._log_validation_event("backup_curvature_used", {"v_ego": v_ego, "timestamp": time.time()})
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
        self._log_validation_event("backup_acceleration_used", {"timestamp": time.time()})
        cloudlog.warning("Backup system: Using neutral acceleration due to primary system failure")
        return 0.0

    def emergency_stop(self) -> Tuple[float, float]:
        """
        Activate emergency stop by returning safe values
        :return: Tuple of (safe_curvature, safe_acceleration) for emergency stop
        """
        self.safety_engaged = True
        self.fallback_active = True
        self._log_validation_event("emergency_stop_activated", {"timestamp": time.time()})
        cloudlog.error("Backup system: Emergency stop activated")
        return (0.0, -3.5)  # Neutral steering, maximum safe deceleration

    def is_system_operational(self) -> bool:
        """
        Check if primary system is operational
        :return: True if primary system appears to be working correctly
        """
        return not self.fallback_active and not self.safety_engaged

    def _log_validation_event(self, event_type: str, details: Dict[str, Any]):
        """Log a validation event for monitoring and analysis"""
        event = {
            'timestamp': time.time(),
            'type': event_type,
            'details': details
        }
        self.validation_history.append(event)
        if len(self.validation_history) > self.max_history:
            self.validation_history.pop(0)


class RedundantControlValidator:
    """
    Provides redundant validation for control outputs using independent calculation
    """

    def __init__(self, enable_steering_rate_validation: bool = True):
        self.backup_system = SafetyBackupSystem()
        self.enable_steering_rate_validation = enable_steering_rate_validation
        self.last_curvature = 0.0  # Track for steering rate validation
        self.last_validation_time = time.time()

    def validate_control_outputs(self, desired_curvature: float, desired_accel: float,
                               v_ego: float, max_lateral_accel: float = 3.0,
                               max_accel: float = 2.0, max_decel: float = -3.5,
                               vehicle_params: Optional[Dict[str, float]] = None) -> Tuple[float, float]:
        """
        Validate control outputs using redundant safety checks
        :param desired_curvature: Desired curvature from primary system
        :param desired_accel: Desired acceleration from primary system
        :param v_ego: Current vehicle speed
        :param max_lateral_accel: Maximum allowed lateral acceleration
        :param max_accel: Maximum allowed positive acceleration
        :param max_decel: Maximum allowed negative acceleration (deceleration)
        :param vehicle_params: Optional vehicle-specific parameters
        :return: Tuple of (validated_curvature, validated_acceleration)
        """
        current_time = time.time()
        dt = current_time - self.last_validation_time
        self.last_validation_time = current_time

        # Validate curvature
        validated_curvature = self.backup_system.validate_curvature(
            desired_curvature, v_ego, max_lateral_accel, vehicle_params
        )

        # Apply steering rate validation if enabled
        if self.enable_steering_rate_validation:
            validated_curvature = self.backup_system.validate_steering_rate(
                validated_curvature, self.last_curvature, dt
            )
            self.last_curvature = validated_curvature
        else:
            self.last_curvature = validated_curvature

        # Validate acceleration
        validated_accel = self.backup_system.validate_acceleration(
            desired_accel, max_accel, max_decel, v_ego, vehicle_params
        )

        return (validated_curvature, validated_accel)

    def validate_and_backup(self, desired_curvature: float, desired_accel: float,
                          v_ego: float, model_confidence: float,
                          environmental_risk: float,
                          vehicle_params: Optional[Dict[str, float]] = None,
                          current_curvature: float = 0.0) -> Tuple[float, float]:
        """
        Perform full validation with backup systems if needed
        :param desired_curvature: Desired curvature from primary system
        :param desired_accel: Desired acceleration from primary system
        :param v_ego: Current vehicle speed
        :param model_confidence: Confidence in model outputs (0.0-1.0)
        :param environmental_risk: Environmental risk assessment (0.0-1.0)
        :param vehicle_params: Optional vehicle-specific parameters
        :param current_curvature: Current curvature for rate validation
        :return: Tuple of (safe_curvature, safe_acceleration)
        """
        # Check if model confidence is too low for safe operation
        if model_confidence < 0.5 or environmental_risk > 0.8:
            cloudlog.warning(f"Low confidence ({model_confidence}) or high risk ({environmental_risk}), using validated outputs")

        # Perform redundant validation using independent calculations
        validated_curvature, validated_accel = self.validate_control_outputs(
            desired_curvature, desired_accel, v_ego,
            vehicle_params=vehicle_params
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

    def get_validation_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about validation events for monitoring
        """
        if not self.backup_system.validation_history:
            return {
                'total_events': 0,
                'recent_events': [],
                'error_count': 0,
                'clamping_count': 0
            }

        recent_events = self.backup_system.validation_history[-20:]  # Last 20 events

        error_count = sum(1 for event in recent_events if 'error' in event['type'])
        clamping_count = sum(1 for event in recent_events if 'clamped' in event['type'])

        return {
            'total_events': len(self.backup_system.validation_history),
            'recent_events': recent_events,
            'error_count': error_count,
            'clamping_count': clamping_count,
            'system_operational': self.backup_system.is_system_operational()
        }