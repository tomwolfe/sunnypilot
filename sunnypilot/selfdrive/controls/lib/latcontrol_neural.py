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
        # Variational Bayesian Blending:
        # Instead of a hard gate, we treat the neural and kinematic torques as
        # probability distributions and perform a variance-weighted fusion.

        # 1. Neural Variance (estimated from uncertainty)
        # We map uncertainty to a variance parameter.
        neural_variance = np.square(lateral_uncertainty) + 0.01

        # 2. Kinematic Variance (estimated based on speed and consistency)
        # At high speeds, kinematics is highly reliable.
        # Base envelope follows speed-based heuristic, scaled by relative uncertainty.
        allowed_diff_base = 0.3 + 0.2 * max(0, 1.0 - CS.vEgo / 20.0)
        kinematic_variance = 0.05 / max(1.0, (CS.vEgo / 15.0)**2)

        # 3. Dynamic Cost Sensitivity: Increase kinematic weight if neural deviates significantly
        # This acts as an "active inference" check.
        torque_diff = abs(torque_neural - fallback_torque)
        innovation_cost = np.square(torque_diff / allowed_diff_base)
        kinematic_variance /= (1.0 + innovation_cost)

        # Bayesian Blend: Weights are inversely proportional to variance
        w_neural = 1.0 / neural_variance
        w_kinematic = 1.0 / kinematic_variance

        total_weight = w_neural + w_kinematic
        output_torque = (w_neural * (torque_neural + self.bias) + w_kinematic * fallback_torque) / total_weight

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
