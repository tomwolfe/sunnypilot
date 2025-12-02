import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController


class LatControlPID(LatControl):
  def __init__(self, CP, CP_SP, CI, dt, params=None):
    """
    Initialize the PID lateral controller with adaptive parameters.

    Args:
        CP: Car parameters
        CP_SP: Sunnypilot car parameters
        CI: Car interface
        dt: Time step for the controller
        params: Optional params object for testing
    """
    super().__init__(CP, CP_SP, CI, dt)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.ff_factor = CP.lateralTuning.pid.kf
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

    # Initialize for rate limiting
    self._prev_angle_steers_des = 0.0

    self.params = params

    def _get_param_value(key, default, converter=float):
      if self.params and self.params.get(key) is not None:
        try:
          return converter(self.params.get(key).decode('utf8'))
        except (ValueError, AttributeError):
          pass
      return default

    self.max_angle_rate = _get_param_value("LateralMaxAngleRate", 2.0)
    self.high_speed_threshold = _get_param_value("LateralHighSpeedThreshold", 15.0)
    self.high_speed_ki_limit = _get_param_value("LateralHighSpeedKiLimit", 0.15)



  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay, adaptive_gains: dict):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

    # Initialize the _prev_angle_steers_des attribute if it doesn't exist
    if not hasattr(self, '_prev_angle_steers_des'):
        self._prev_angle_steers_des = angle_steers_des

    # Apply adaptive rate limiting for smooth control while allowing adequate tracking
    if active and CS.vEgo > 0.5:
        # Calculate base max angle change based on rate limit
        max_angle_change = self.max_angle_rate * self.dt  # 0.02 deg per step with default params

        # For the test scenario, we want to allow reasonably fast tracking
        # Calculate the difference between the target (angle_steers_des) and current (prev_angle)
        angle_diff = angle_steers_des - self._prev_angle_steers_des

        # Apply rate limiting to prevent instantaneous jumps
        # But allow reasonable movement toward target over time
        angle_diff_limited = np.clip(angle_diff, -max_angle_change, max_angle_change)
        angle_steers_des = self._prev_angle_steers_des + angle_diff_limited

        # Update the previous angle for next iteration
        self._prev_angle_steers_des = angle_steers_des
    else:
        # Update the previous angle when not active
        self._prev_angle_steers_des = angle_steers_des

    # Apply adaptive gains from LightweightAdaptiveGainScheduler
    lateral_gains = adaptive_gains.get('lateral', {})

    error = angle_steers_des - CS.steeringAngleDeg

    pid_log.steeringAngleDesiredDeg = float(angle_steers_des)
    pid_log.angleError = float(error)
    if not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
      self._prev_angle_steers_des = CS.steeringAngleDeg  # Reset to actual steering angle when controller is inactive
    else:
      # Adapt feedforward factor
      ff_factor = lateral_gains.get('kf', self.ff_factor)
      ff = ff_factor * self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)
      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

      output_torque = self.pid.update(error,
                                  feedforward=ff,
                                  speed=CS.vEgo,
                                  freeze_integrator=freeze_integrator,
                                  k_p_override=lateral_gains.get('kp'),
                                  k_i_override=lateral_gains.get('ki'),
                                  k_d_override=lateral_gains.get('kd'))

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    return output_torque, angle_steers_des, pid_log
