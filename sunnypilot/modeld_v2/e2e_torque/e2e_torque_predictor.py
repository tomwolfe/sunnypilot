"""
Pure E2E Torque Prediction Module for sunnypilot
=================================================

This module implements direct torque prediction from the neural network,
eliminating the need for MPC/PID middleman for lateral control.

Key Features:
- Direct torque output from policy model
- Uncertainty-aware torque blending
- Self-healing bias correction
- Integration with existing safety systems
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class E2ETorqueOutput:
    """Output from the E2E torque prediction model"""
    torque: float
    torque_steering: float
    torque_drive: float
    uncertainty: float
    confidence: float
    is_valid: bool


class E2ETorquePredictor:
    """
    Pure E2E Torque Prediction
    
    This class handles direct torque prediction from neural network outputs,
    bypassing the traditional MPC/PID control loop.
    
    The model outputs:
    - torque_steering: Direct steering torque (Nm)
    - torque_drive: Direct drive torque/acceleration (Nm for torque, m/s^2 for accel)
    - uncertainty: Standard deviation of the prediction
    """
    
    def __init__(self, 
                 bias_time_constant: float = 30.0,
                 min_confidence: float = 0.3,
                 max_torque_rate: float = 500.0):
        self.bias = 0.0
        self.bias_time_constant = bias_time_constant
        self.min_confidence = min_confidence
        self.max_torque_rate = max_torque_rate
        
        self._bias_filter_state = 0.0
        self._prev_torque = 0.0
        
    def compute_torque_from_accel(self, 
                                   desired_accel: float,
                                   v_ego: float,
                                   torque_from_lateral_accel_fn=None) -> float:
        """
        Convert desired longitudinal acceleration to torque
        
        This is a fallback for when direct torque prediction is not available
        """
        if torque_from_lateral_accel_fn is not None:
            desired_lat_accel = desired_accel * 0.0
            return torque_from_lateral_accel_fn(desired_lat_accel)
        return desired_accel * 1000.0
        
    def update_bias(self, actual_curvature: float, v_ego: float, 
                   torque_neural: float, speed_threshold: float = 10.0,
                   dt: float = 0.05):
        """
        Online learning / residual adaptation
        
        Update the bias by comparing neural prediction to estimated required torque.
        Uses exponential moving average with configurable time constant.
        """
        if v_ego < speed_threshold:
            return
            
        estimated_actual_torque = self._estimate_required_torque(actual_curvature, v_ego)
        
        if estimated_actual_torque is not None:
            bias_update = estimated_actual_torque - torque_neural
            alpha = dt / self.bias_time_constant
            self.bias = self.bias * (1 - alpha) + bias_update * alpha
            
    def _estimate_required_torque(self, curvature: float, v_ego: float) -> Optional[float]:
        """
        Estimate the actual required torque based on observed vehicle response
        
        This would ideally come from steering angle sensors and lateral acceleration
        """
        if curvature == 0.0 or v_ego == 0.0:
            return None
            
        lateral_accel = curvature * (v_ego ** 2)
        return lateral_accel * 50.0
        
    def process_raw_output(self, 
                          torque_output: np.ndarray,
                          uncertainty_output: Optional[np.ndarray] = None,
                          apply_bias: bool = True) -> E2ETorqueOutput:
        """
        Process raw neural network output into usable torque command
        
        Args:
            torque_output: Raw torque predictions from model [batch, time]
            uncertainty_output: Uncertainty/std dev predictions
            apply_bias: Whether to apply the learned bias correction
            
        Returns:
            E2ETorqueOutput with processed torque and metadata
        """
        if torque_output is None or len(torque_output.flatten()) == 0:
            return E2ETorqueOutput(
                torque=0.0, torque_steering=0.0, torque_drive=0.0,
                uncertainty=1.0, confidence=0.0, is_valid=False
            )
            
        torque = float(torque_output[0, 0] if torque_output.ndim > 1 else torque_output[0])
        
        if apply_bias:
            torque = torque + self.bias
            
        uncertainty = 0.1
        if uncertainty_output is not None:
            uncertainty = float(uncertainty_output[0, 0] if uncertainty_output.ndim > 1 else uncertainty_output[0])
            
        confidence = self._uncertainty_to_confidence(uncertainty)
        
        torque_rate = abs(torque - self._prev_torque)
        if torque_rate > self.max_torque_rate:
            torque = self._prev_torque + np.sign(torque - self._prev_torque) * self.max_torque_rate
            
        self._prev_torque = torque
        
        return E2ETorqueOutput(
            torque=torque,
            torque_steering=torque,
            torque_drive=0.0,
            uncertainty=uncertainty,
            confidence=confidence,
            is_valid=True
        )
        
    def _uncertainty_to_confidence(self, uncertainty: float) -> float:
        """Convert uncertainty to confidence score [0, 1]"""
        return float(np.clip(1.0 - uncertainty / 2.0, 0.0, 1.0))
        
    def blend_with_fallback(self, 
                           e2e_torque: float,
                           fallback_torque: float,
                           confidence: float) -> float:
        """
        Blend E2E torque with fallback (physics-based) torque
        
        Uses confidence-gated blending:
        - High confidence: Use mostly E2E torque
        - Low confidence: Blend towards fallback
        - Very low confidence: Use fallback only
        """
        if confidence > 0.8:
            return e2e_torque
        elif confidence > self.min_confidence:
            blend_weight = (confidence - self.min_confidence) / (0.8 - self.min_confidence)
            return blend_weight * e2e_torque + (1 - blend_weight) * fallback_torque
        else:
            return fallback_torque


class E2ETorqueSafety:
    """
    Safety monitor for E2E torque outputs
    
    Provides hardware-level safety checking of neural network torque outputs
    """
    
    def __init__(self,
                 max_steering_torque: float = 300.0,
                 max_torque_rate: float = 500.0,
                 latency_threshold_ms: float = 100.0):
        self.max_steering_torque = max_steering_torque
        self.max_torque_rate = max_torque_rate
        self.latency_threshold_ms = latency_threshold_ms
        
        self._prev_torque = 0.0
        self._consecutive_failures = 0
        self._safety_state = "nominal"
        
    def validate_torque(self, 
                       torque: float,
                       curvature_estimate: float = 0.0,
                       v_ego: float = 0.0) -> tuple[bool, str]:
        """
        Validate that torque is within safe bounds
        
        Returns:
            (is_valid, reason)
        """
        if abs(torque) > self.max_steering_torque:
            self._consecutive_failures += 1
            self._safety_state = "torque_exceeded"
            return False, f"Torque {torque} exceeds max {self.max_steering_torque}"
            
        torque_rate = abs(torque - self._prev_torque)
        if torque_rate > self.max_torque_rate:
            self._consecutive_failures += 1
            self._safety_state = "rate_exceeded"
            return False, f"Torque rate {torque_rate} exceeds max"
            
        if v_ego > 0:
            expected_lat_accel = abs(curvature_estimate) * (v_ego ** 2)
            actual_lat_accel = abs(torque) / 50.0
            if actual_lat_accel > expected_lat_accel * 5.0:
                self._consecutive_failures += 1
                self._safety_state = "physics_violation"
                return False, "Torque violates physics constraints"
                
        self._consecutive_failures = max(0, self._consecutive_failures - 1)
        if self._consecutive_failures == 0:
            self._safety_state = "nominal"
            
        self._prev_torque = torque
        return True, "ok"
        
    def get_safety_state(self) -> str:
        """Get current safety state"""
        return self._safety_state
