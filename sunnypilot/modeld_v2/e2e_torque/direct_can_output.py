"""
Direct CAN Bus Output Module for E2E Neural Control
====================================================

This module enables direct output of neural network torque commands to the CAN bus,
eliminating the PID controller middleman for true end-to-end control.

Key Features:
- Direct voltage/torque output to CAN
- Rate limiting and safety bounds
- Integration with CBF safety constraints
- Smooth interpolation between neural commands
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class DirectCANOutput:
    """Direct CAN output values for neural control"""
    steering_voltage: float
    steering_current: float
    torque_command: float
    is_safety_limited: bool
    limited_reason: str


class DirectCANOutputter:
    """
    Direct CAN Output for Neural Lateral Control
    
    Outputs neural network commands directly to CAN bus without PID intermediate.
    This enables true end-to-end control where the policy model outputs
    voltage/current/torque directly.
    """
    
    def __init__(self,
                 max_voltage: float = 12.0,
                 max_current: float = 10.0,
                 max_torque: float = 2.0,
                 rate_limit_v_per_s: float = 50.0,
                 interpolation_factor: float = 0.3):
        self.max_voltage = max_voltage
        self.max_current = max_current
        self.max_torque = max_torque
        self.rate_limit_v_per_s = rate_limit_v_per_s
        self.interpolation_factor = interpolation_factor
        
        self._prev_voltage = 0.0
        self._prev_current = 0.0
        self._prev_torque = 0.0
        self._safety_override = False
        self._override_reason = ""
        
    def compute_direct_output(self,
                              neural_torque: float,
                              v_ego: float,
                              lateral_uncertainty: float,
                              dt: float = 0.05) -> DirectCANOutput:
        """
        Compute direct CAN output from neural torque prediction
        
        Args:
            neural_torque: Direct torque from neural network (Nm)
            v_ego: Current vehicle speed (m/s)
            lateral_uncertainty: Uncertainty from model (0-1)
            dt: Time step
            
        Returns:
            DirectCANOutput with clamped and limited values
        """
        is_limited = False
        limited_reason = ""
        
        torque_limited = self._apply_torque_limits(neural_torque, v_ego)
        if abs(torque_limited) < abs(neural_torque):
            is_limited = True
            limited_reason = "torque_limit"
            
        voltage = self._torque_to_voltage(torque_limited, v_ego)
        current = self._torque_to_current(torque_limited)
        
        voltage_rate = abs(voltage - self._prev_voltage) / dt
        if voltage_rate > self.rate_limit_v_per_s:
            voltage = self._prev_voltage + np.sign(voltage - self._prev_voltage) * self.rate_limit_v_per_s * dt
            is_limited = True
            limited_reason = "rate_limit"
            
        voltage = self._apply_uncertainty_scaling(voltage, lateral_uncertainty)
        
        if self._safety_override:
            voltage = 0.0
            current = 0.0
            torque_limited = 0.0
            is_limited = True
            limited_reason = self._override_reason
            
        self._prev_voltage = voltage
        self._prev_current = current
        self._prev_torque = torque_limited
        
        return DirectCANOutput(
            steering_voltage=voltage,
            steering_current=current,
            torque_command=torque_limited,
            is_safety_limited=is_limited,
            limited_reason=limited_reason
        )
        
    def _torque_to_voltage(self, torque: float, v_ego: float) -> float:
        """
        Convert torque command to steering motor voltage
        
        Uses a simple model: V = k * torque / speed
        At higher speeds, less voltage is needed for same torque
        """
        if abs(v_ego) < 0.1:
            return np.sign(torque) * self.max_voltage * 0.5
            
        k = 0.5
        voltage = k * torque / (v_ego / 10.0)
        
        return np.clip(voltage, -self.max_voltage, self.max_voltage)
        
    def _torque_to_current(self, torque: float) -> float:
        """Convert torque to motor current (simplified)"""
        current = torque * 2.0
        return np.clip(current, -self.max_current, self.max_current)
        
    def _apply_torque_limits(self, torque: float, v_ego: float) -> float:
        """Apply safety limits to torque command"""
        torque_limited = np.clip(torque, -self.max_torque, self.max_torque)
        
        if v_ego > 25.0:
            torque_limited *= 0.7
        elif v_ego > 15.0:
            torque_limited *= 0.85
            
        return torque_limited
        
    def _apply_uncertainty_scaling(self, voltage: float, uncertainty: float) -> float:
        """
        Scale output based on model uncertainty
        
        Higher uncertainty = scale back neural command
        This provides a soft safety margin
        """
        if uncertainty < 0.3:
            return voltage
        elif uncertainty < 0.6:
            scale = 1.0 - (uncertainty - 0.3) * 1.0
            return voltage * scale
        else:
            return voltage * 0.4
            
    def set_safety_override(self, override: bool, reason: str = ""):
        """Set safety override to zero all outputs"""
        self._safety_override = override
        self._override_reason = reason
        
    def reset(self):
        """Reset state"""
        self._prev_voltage = 0.0
        self._prev_current = 0.0
        self._prev_torque = 0.0
        self._safety_override = False
        self._override_reason = ""


class CANMessageBuilder:
    """
    Builds CAN messages for direct neural control output
    """
    
    STEERING_CMD_ID = 0x1A4
    TORQUE_CMD_ID = 0x1A5
    
    @staticmethod
    def build_steering_message(voltage: float, current: float) -> bytes:
        """Build CAN message for steering control"""
        voltage_int = int(np.clip(voltage / 12.0 * 2048, -2048, 2047))
        current_int = int(np.clip(current / 10.0 * 512, -512, 511))
        
        data = bytearray(8)
        data[0] = voltage_int & 0xFF
        data[1] = (voltage_int >> 8) & 0xFF
        data[2] = current_int & 0xFF
        data[3] = (current_int >> 8) & 0xFF
        
        return bytes(data)
        
    @staticmethod
    def build_torque_message(torque: float) -> bytes:
        """Build CAN message for torque command"""
        torque_int = int(np.clip(torque / 2.0 * 32768, -32768, 32767))
        
        data = bytearray(8)
        data[0] = torque_int & 0xFF
        data[1] = (torque_int >> 8) & 0xFF
        
        return bytes(data)
