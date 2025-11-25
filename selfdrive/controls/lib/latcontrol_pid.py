import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.params import Params


class LatControlPID(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.ff_factor = CP.lateralTuning.pid.kf
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

    # Load configurable parameters
    params = Params()
    self.max_angle_rate = float(params.get("LateralMaxAngleRate", encoding='utf8') or "2.0")  # degrees per second
    self.high_speed_threshold = float(params.get("LateralHighSpeedThreshold", encoding='utf8') or "15.0")  # m/s
    self.high_speed_ki_limit = float(params.get("LateralHighSpeedKiLimit", encoding='utf8') or "0.15")

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

    # Apply smoothing to reduce abrupt steering angle changes
    # Limit the rate of change of desired steering angle
    if hasattr(self, '_prev_angle_steers_des'):
      max_angle_rate = self.max_angle_rate  # degrees per second - now configurable
      rate_limit = max_angle_rate * self.dt
      angle_diff = angle_steers_des - self._prev_angle_steers_des
      angle_diff_limited = np.clip(angle_diff, -rate_limit, rate_limit)
      angle_steers_des = self._prev_angle_steers_des + angle_diff_limited
    self._prev_angle_steers_des = angle_steers_des

    error = angle_steers_des - CS.steeringAngleDeg

    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = error
    if not active:
      output_torque = 0.0
      pid_log.active = False

    else:
      # offset does not contribute to resistive torque
      ff = self.ff_factor * self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)
      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

      # Adaptive PID parameters for smoother response
      # Reduce integral gain at high speed to prevent oscillations
      if CS.vEgo > self.high_speed_threshold:  # Above configurable threshold (default 15 m/s or 54 km/h)
        # Create temporary PID with reduced integral gain
        temp_ki = min(self.pid.ki, self.high_speed_ki_limit)  # Reduce ki to reduce oscillation - now configurable
        temp_pid = PIDController((self.CP.lateralTuning.pid.kpBP, self.CP.lateralTuning.pid.kpV),
                                 (self.CP.lateralTuning.pid.kiBP, [temp_ki] * len(self.CP.lateralTuning.pid.kiV)),
                                 pos_limit=self.steer_max, neg_limit=-self.steer_max)
        temp_pid.error_integral = self.pid.error_integral  # Preserve integral term
        output_torque = temp_pid.update(error,
                                  feedforward=ff,
                                  speed=CS.vEgo,
                                  freeze_integrator=freeze_integrator)
        # Preserve the original PID's integral term for next iteration
        self.pid.error_integral = temp_pid.error_integral
      else:
        output_torque = self.pid.update(error,
                                  feedforward=ff,
                                  speed=CS.vEgo,
                                  freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    return output_torque, angle_steers_des, pid_log
