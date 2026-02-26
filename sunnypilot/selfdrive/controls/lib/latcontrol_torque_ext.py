"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Direct Torque E2E: When enabled, the model outputs raw torque commands directly,
bypassing the curvature→PID chain. This allows true end-to-end learning of the
relationship between visual input and steering torque.
"""

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride


class LatControlTorqueExt(NeuralNetworkLateralControl, LatControlTorqueExtOverride):
  def __init__(self, lac_torque, CP, CP_SP, CI):
    NeuralNetworkLateralControl.__init__(self, lac_torque, CP, CP_SP, CI)
    LatControlTorqueExtOverride.__init__(self, CP)
    
    # Direct Torque E2E mode - bypasses curvature→PID chain
    self.params = Params()
    self.direct_torque_enabled = self.params.get_bool("DirectTorqueE2E")
    self.direct_torque_available = False  # Set when model outputs direct_torque
    self._direct_torque_output = 0.0
    
    # Blending factor for smooth transition between traditional and direct torque
    # 0.0 = traditional NNFF+PID, 1.0 = pure direct torque
    self.direct_torque_blend = 0.0

  def update(self, CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
             desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
             desired_curvature, actual_curvature, steer_limited_by_safety, output_torque, model_v2=None):
    self._ff = ff
    self._pid = pid
    self._pid_log = pid_log
    self._setpoint = setpoint
    self._measurement = measurement
    self._roll_compensation = roll_compensation
    self._lateral_accel_deadzone = lateral_accel_deadzone
    self._desired_lateral_accel = desired_lateral_accel
    self._actual_lateral_accel = actual_lateral_accel
    self._desired_curvature = desired_curvature
    self._actual_curvature = actual_curvature
    self._gravity_adjusted_lateral_accel = gravity_adjusted_lateral_accel
    self._steer_limited_by_safety = steer_limited_by_safety
    self._output_torque = output_torque

    self.update_calculations(CS, VM, desired_lateral_accel)
    
    # Check if direct torque output is available from model
    self.direct_torque_available = (model_v2 is not None and 
                                    hasattr(model_v2, 'direct_torque') and 
                                    model_v2.direct_torque is not None)
    
    # Update direct torque blend factor based on availability and user setting
    self._update_direct_torque_blend(CS)
    
    # Apply direct torque if enabled and available
    if self.direct_torque_enabled and self.direct_torque_available and self.direct_torque_blend > 0.5:
      self._apply_direct_torque(model_v2, CS)
    else:
      # Traditional NNFF + PID control
      self.update_neural_network_feedforward(CS, params, calibrated_pose)

    return self._pid_log, self._output_torque
  
  def _update_direct_torque_blend(self, CS):
    """Smoothly blend between traditional and direct torque control."""
    target_blend = 1.0 if (self.direct_torque_enabled and self.direct_torque_available) else 0.0
    
    # Blend over 0.5 seconds for smooth transitions
    blend_rate = 0.02  # ~2% per 20ms step
    if target_blend > self.direct_torque_blend:
      self.direct_torque_blend = min(target_blend, self.direct_torque_blend + blend_rate)
    else:
      self.direct_torque_blend = max(target_blend, self.direct_torque_blend - blend_rate)
    
    # Disable direct torque at very low speeds (< 5 m/s) for safety
    if CS.vEgo < 5.0:
      self.direct_torque_blend = 0.0

  def _apply_direct_torque(self, model_v2, CS):
    """
    Apply direct torque output from the model, bypassing the curvature→PID chain.
    
    The model outputs raw torque commands learned from data, which already
    incorporate friction, inertia, and vehicle-specific dynamics.
    """
    # Extract direct torque from model output (shape: [1, 1])
    self._direct_torque_output = float(model_v2.direct_torque[0, 0])
    
    # Apply blending with traditional control for safety
    # When blend=1.0, use 100% direct torque
    # When blend<1.0, blend with traditional PID output
    if self.direct_torque_blend < 1.0 and self._pid_log is not None:
      traditional_torque = self._pid_log.output
      self._output_torque = (self.direct_torque_blend * self._direct_torque_output + 
                            (1.0 - self.direct_torque_blend) * traditional_torque)
    else:
      self._output_torque = self._direct_torque_output
    
    # Apply safety limits (same as traditional control)
    self._output_torque = float(max(-self.lac_torque.steer_max, 
                                   min(self.lac_torque.steer_max, self._output_torque)))
    
    # Update PID log for monitoring (even though we're bypassing PID)
    if self._pid_log is not None:
      self._pid_log.output = self._output_torque
      self._pid_log.outputSat = 0.0  # No saturation in direct torque mode
