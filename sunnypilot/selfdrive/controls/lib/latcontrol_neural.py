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

    # Hysteresis Gate: Filtered neural weight for smooth transitions
    self.neural_weight_filtered = FirstOrderFilter(1.0, 0.2, dt)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay, torque_neural=0.0, lateral_uncertainty=0.0, actual_curvature=0.0, e2e_weight=0.0, desire=log.Desire.none):
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active or desire == log.Desire.pauseLateral:
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
        # Neural Authority Dominance:
        # Instead of blending based on fixed certainty, we adopt an "Optimistic Neural" stance.
        # We assume the model is correct unless it violates high-confidence physical constraints.
        
        # 1. Neural Weight from Latent Uncertainty
        # Using a softer sigmoid to allow more "neural character" even at moderate uncertainty.
        sigmoid_center = 0.55  # Increased from 0.35 to keep neural active longer
        sigmoid_steepness = 10.0
        w_neural_target = 1.0 / (1.0 + np.exp(sigmoid_steepness * (lateral_uncertainty - sigmoid_center)))
        w_neural_gated = self.neural_weight_filtered.update(w_neural_target)

        # 2. Dynamic Constraint Envelope:
        # We calculate the "allowed" deviation from physics. In pure E2E, this envelope is wide.
        # It narrows only at very high speeds or high lateral G-loads.
        lat_g = abs(actual_curvature * CS.vEgo ** 2)
        constraint_tightness = interp(lat_g, [0.0, 3.0, 5.0], [0.1, 0.4, 0.8])
        allowed_diff = (0.5 + 0.5 * max(0, 1.0 - CS.vEgo / 25.0)) * (1.0 - constraint_tightness)

        # 3. Direct Neural Drive with Bayesian Guarding:
        # If the difference between neural and fallback is within allowed_diff, we use 100% neural.
        # We only blend back to kinematics if the model deviates beyond physical plausibility.
        torque_diff = abs(torque_neural - fallback_torque)
        out_of_bounds = max(0.0, (torque_diff - allowed_diff) / 0.2)
        
        # Final Blend Weight: Mix neural intent with safety-bound out_of_bounds
        w_final_neural = w_neural_gated * (1.0 - min(1.0, out_of_bounds))
        
        # Apply Self-Healing Bias to the Neural signal
        output_torque = w_final_neural * (torque_neural + self.bias) + (1.0 - w_final_neural) * fallback_torque

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
