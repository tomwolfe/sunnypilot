"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log
import numpy as np
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl


class LatControlNeural(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()

    # Self-Healing: Bias-correction layer with 30-second time constant
    self.bias = 0.0
    self.bias_filter = FirstOrderFilter(0.0, 30.0, dt)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay, torque_neural=0.0, lateral_uncertainty=0.0, actual_curvature=0.0):
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # Calculate fallback torque
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      fallback_torque = self.torque_from_lateral_accel(desired_lateral_accel, self.CP.lateralTuning.torque)

      # Use torque directly from the neural model if available
      # Gated approach:
      # 1. Full neural authority when uncertainty is low (< GATED_THRESHOLD)
      # 2. Linear blend to fallback between GATED_THRESHOLD and UNCERTAINTY_THRESHOLD
      # 3. Safety gate: If neural torque is significantly different from kinematic expected torque,
      #    increase blend towards fallback for safety.
      GATED_THRESHOLD = 0.15
      UNCERTAINTY_THRESHOLD = 0.5

      if torque_neural != 0.0:
        # Initial blend factor based on model uncertainty
        blend_factor = np.clip((lateral_uncertainty - GATED_THRESHOLD) / (UNCERTAINTY_THRESHOLD - GATED_THRESHOLD), 0.0, 1.0)

        # UQ Envelope: Replace static allowed_diff with an uncertainty-quantified envelope.
        # Certain (low uncertainty) -> Tight envelope (smaller allowed_diff)
        # Uncertain (high uncertainty) -> Wide envelope (larger allowed_diff)
        # This follows the principle: If the model is certain, the envelope is tight; if the model is uncertain, the envelope widens.
        # Base envelope follows speed-based heuristic, scaled by relative uncertainty.
        allowed_diff_base = 0.3 + 0.2 * max(0, 1.0 - CS.vEgo / 20.0)
        uncertainty_scale = max(0.33, lateral_uncertainty / GATED_THRESHOLD)
        allowed_diff = allowed_diff_base * uncertainty_scale

        # Safety Sanity Check: If neural torque exceeds fallback by a large margin, it might be a hallucination.
        # We allow more divergence at low speeds, but tighten at high speeds.
        torque_diff = abs(torque_neural - fallback_torque)
        if torque_diff > allowed_diff:
          safety_blend = np.clip((torque_diff - allowed_diff) / 0.2, 0.0, 1.0)
          blend_factor = max(blend_factor, safety_blend)

        output_torque = (1.0 - blend_factor) * (torque_neural + self.bias) + blend_factor * fallback_torque

        # Online Learning / Residual Adaptation:
        # Update bias by comparing neural prediction to estimated required torque.
        # We only update when speed is high enough and no user intervention.
        if not CS.steeringPressed and CS.vEgo > 10.0:
          # Estimate the "true" required torque for the current actual curvature
          actual_lateral_accel = actual_curvature * CS.vEgo ** 2
          actual_torque = self.torque_from_lateral_accel(actual_lateral_accel, self.CP.lateralTuning.torque)

          # The bias is the difference between physics-estimated actual torque and our neural prediction
          # We filter this over 30 seconds for a stable "self-healing" correction.
          current_bias = actual_torque - torque_neural
          self.bias = self.bias_filter.update(current_bias)
      else:
        output_torque = fallback_torque

      pid_log.active = True
      pid_log.p = 0.0
      pid_log.i = self.bias
      pid_log.d = 0.0
      pid_log.f = output_torque
      pid_log.output = float(-output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    return -output_torque, 0.0, pid_log
