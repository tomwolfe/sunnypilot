"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log
import numpy as np
from openpilot.selfdrive.controls.lib.latcontrol import LatControl


class LatControlNeural(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay, torque_neural=0.0, lateral_uncertainty=0.0):
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # Calculate fallback torque
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      fallback_torque = self.torque_from_lateral_accel(desired_lateral_accel, self.CP.lateralTuning.torque)

      # Use torque directly from the neural model if available
      # Blend based on uncertainty: 0.0 (certain) -> torque_neural, > threshold (uncertain) -> fallback_torque
      # Standard deviation of 0.5 meters is used as a threshold for high uncertainty
      UNCERTAINTY_THRESHOLD = 0.5
      blend_factor = np.clip(lateral_uncertainty / UNCERTAINTY_THRESHOLD, 0.0, 1.0)

      if torque_neural != 0.0:
        output_torque = (1.0 - blend_factor) * torque_neural + blend_factor * fallback_torque
      else:
        output_torque = fallback_torque

      pid_log.active = True
      pid_log.p = 0.0
      pid_log.i = 0.0
      pid_log.d = 0.0
      pid_log.f = output_torque
      pid_log.output = float(-output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    return -output_torque, 0.0, pid_log
