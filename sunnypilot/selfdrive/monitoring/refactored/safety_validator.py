"""
Safety Validator for sunnypilot - Validates parameter changes and ensures safety constraints

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import logging
from typing import Dict, Any, Optional


class SafetyValidator:
    """
    Validates parameter changes to ensure safety constraints are met
    Addresses the critical concern about self-tuning systems without proper validation
    """
    def __init__(self):
        # Define safety constraints for various tuning parameters
        self.safety_constraints = {
            'lateral_kp_factor': {
                'min': 0.3,
                'max': 2.0,
                'default': 1.0,
                'description': 'Lateral proportional gain factor'
            },
            'lateral_ki_factor': {
                'min': 0.2,
                'max': 1.5,
                'default': 1.0,
                'description': 'Lateral integral gain factor'
            },
            'longitudinal_accel_limit_factor': {
                'min': 0.5,
                'max': 1.8,
                'default': 1.0,
                'description': 'Longitudinal acceleration limit factor'
            },
            'model_confidence_threshold': {
                'min': 0.3,
                'max': 0.9,
                'default': 0.7,
                'description': 'Minimum model confidence threshold'
            },
            'max_lateral_acceleration': {
                'min': 1.0,
                'max': 4.0,
                'default': 2.0,
                'description': 'Maximum allowed lateral acceleration'          
            }
        }
        
        # Rate limits for parameter changes to prevent instability
        self.change_rate_limits = {
            'lateral_kp_factor': 0.05,  # Max 5% change per validation
            'lateral_ki_factor': 0.03,  # Max 3% change per validation
            'longitudinal_accel_limit_factor': 0.05,  # Max 5% change per validation
        }

        # Store last validated values to enforce rate limits
        self.last_validated_values = {}

    def validate_parameter_change(self, parameter_name: str, proposed_value: float, current_value: float = None) -> tuple[bool, float, str]:
        """
        Validate a proposed parameter change against safety constraints
        
        Returns: (is_valid, validated_value, reason)
        """
        if parameter_name not in self.safety_constraints:
            return False, current_value or self.safety_constraints[parameter_name]['default'], \
                   f"Unknown parameter: {parameter_name}"

        constraints = self.safety_constraints[parameter_name]
        
        # Check absolute bounds
        if proposed_value < constraints['min']:
            return False, current_value or constraints['default'], \
                   f"Value {proposed_value} below minimum {constraints['min']} for {parameter_name}"
        
        if proposed_value > constraints['max']:
            return False, current_value or constraints['default'], \
                   f"Value {proposed_value} above maximum {constraints['max']} for {parameter_name}"
        
        # Check rate of change if current value is provided
        if current_value is not None and parameter_name in self.change_rate_limits:
            max_change = abs(current_value) * self.change_rate_limits[parameter_name]
            actual_change = abs(proposed_value - current_value)
            
            if actual_change > max_change:
                # Clamp to the allowed rate of change
                if proposed_value > current_value:
                    clamped_value = current_value + max_change
                else:
                    clamped_value = current_value - max_change
                
                clamped_value = max(constraints['min'], min(constraints['max'], clamped_value))
                
                logging.warning(f"Parameter {parameter_name} change rate limited from {proposed_value} to {clamped_value}")
                return True, clamped_value, f"Change rate limited to {max_change}"
        
        # Additional domain-specific validations
        if parameter_name == 'model_confidence_threshold' and proposed_value < 0.5:
            logging.warning(f"Setting {parameter_name} below 0.5 may compromise safety")
        
        return True, proposed_value, "Validated successfully"

    def validate_all_parameters(self, proposed_params: Dict[str, float], current_params: Dict[str, float] = None) -> tuple[bool, Dict[str, float], Dict[str, str]]:
        """
        Validate multiple parameters at once
        
        Returns: (all_valid, validated_params, reasons)
        """
        validated_params = {}
        reasons = {}
        all_valid = True
        
        for param_name, proposed_value in proposed_params.items():
            current_value = current_params.get(param_name) if current_params else None
            is_valid, validated_value, reason = self.validate_parameter_change(param_name, proposed_value, current_value)
            
            if is_valid:
                validated_params[param_name] = validated_value
            else:
                all_valid = False
                validated_params[param_name] = current_value if current_value is not None else self.safety_constraints[param_name]['default']
            
            reasons[param_name] = reason
            
        return all_valid, validated_params, reasons

    def validate_performance_based_changes(self, performance_metrics: Dict[str, Any], current_params: Dict[str, float]) -> Dict[str, float]:
        """
        Apply performance-based parameter adjustments with safety validation
        This implements the "human-in-the-loop" approach by requiring clear performance degradation
        before allowing parameter changes
        """
        suggested_changes = {}
        
        # Only make changes if there's clear performance degradation
        lat_acc_mean = performance_metrics.get('lateral_accuracy', {}).get('mean', 0.15)
        long_acc_mean = performance_metrics.get('longitudinal_accuracy', {}).get('mean', 0.3)
        comfort_mean = performance_metrics.get('ride_comfort', {}).get('mean', 0.8)
        
        # Define clear thresholds for when changes are justified
        lat_degraded = lat_acc_mean > 0.25  # Only adjust if lateral error > 0.25m
        long_degraded = long_acc_mean > 0.5  # Only adjust if longitudinal error > 0.5m
        comfort_degraded = comfort_mean < 0.6  # Only adjust if comfort < 0.6

        if lat_degraded:
            # Increase lateral gains to improve tracking, but within safety bounds
            current_lateral_kp = current_params.get('lateral_kp_factor', 1.0)
            suggested_changes['lateral_kp_factor'] = min(current_lateral_kp * 1.05, 1.5)  # Max 5% increase per cycle

        if long_degraded:
            # Be very conservative with longitudinal changes
            current_accel_limit = current_params.get('longitudinal_accel_limit_factor', 1.0)
            suggested_changes['longitudinal_accel_limit_factor'] = max(current_accel_limit * 0.98, 0.7)  # Max 2% decrease per cycle

        if comfort_degraded:
            # Reduce gains to improve comfort
            current_lateral_kp = current_params.get('lateral_kp_factor', 1.0)
            suggested_changes['lateral_kp_factor'] = max(current_lateral_kp * 0.98, 0.7)  # Max 2% decrease per cycle

        # Validate all suggested changes
        valid, validated_changes, reasons = self.validate_all_parameters(suggested_changes, current_params)
        
        if not valid:
            logging.warning(f"Performance-based parameter changes rejected: {reasons}")
            return {}
        
        return validated_changes

    def need_human_approval(self, proposed_changes: Dict[str, float], current_params: Dict[str, float]) -> tuple[bool, str]:
        """
        Determine if proposed changes require human approval based on risk level
        
        Returns: (needs_approval, reason)
        """
        for param_name, proposed_value in proposed_changes.items():
            current_value = current_params.get(param_name, self.safety_constraints[param_name]['default'])
            
            # Calculate percentage change
            percent_change = abs((proposed_value - current_value) / current_value) * 100
            
            # Require approval for large changes
            if percent_change > 20:  # More than 20% change
                return True, f"Large change (>20%) in {param_name}: {current_value} -> {proposed_value}"
                
            # Require approval for safety-critical parameters with any significant change
            if param_name in ['model_confidence_threshold', 'max_lateral_acceleration'] and percent_change > 5:
                return True, f"Safety-critical parameter {param_name} change >5%: {current_value} -> {proposed_value}"
        
        return False, "Changes within acceptable limits"