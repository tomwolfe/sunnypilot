import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.params import Params


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

    # Load configurable parameters to allow user customization of vehicle behavior
    if params is None:
      params = Params()
    self.max_angle_rate = self._validate_parameter(
        float(params.get("LateralMaxAngleRate") or "2.0"),
        0.1, 10.0, "LateralMaxAngleRate"
    )  # degrees per second
    self.high_speed_threshold = self._validate_parameter(
        float(params.get("LateralHighSpeedThreshold") or "15.0"),
        5.0, 30.0, "LateralHighSpeedThreshold"
    )  # m/s
    self.high_speed_ki_limit = self._validate_parameter(
        float(params.get("LateralHighSpeedKiLimit") or "0.12"),
        0.01, 0.5, "LateralHighSpeedKiLimit"
    )  # Integral gain limit at high speeds

  def _validate_parameter(self, value, min_val, max_val, name):
    """
    Validate that a parameter is within safe bounds.

    Args:
        value: Parameter value to validate
        min_val: Minimum allowed value
        max_val: Maximum allowed value
        name: Name of the parameter for logging

    Returns:
        Validated parameter value within bounds
    """
    if value < min_val:
      print(f"Warning: {name} value {value} below minimum {min_val}, clamping to minimum")
      return min_val
    elif value > max_val:
      print(f"Warning: {name} value {value} above maximum {max_val}, clamping to maximum")
      return max_val
    return value

  def _restore_original_pid_gains(self, original_kp_values, original_ki_values):
    """
    Restores the original PID gain values after temporary modifications.
    """
    self.pid._k_p = (self.pid._k_p[0], original_kp_values)
    self.pid._k_i = (self.pid._k_i[0], original_ki_values)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

    # Apply advanced smoothing to reduce abrupt steering angle changes
    # Use both rate limiting and low-pass filtering for smoother response
    if hasattr(self, '_prev_angle_steers_des'):
      max_angle_rate = self.max_angle_rate  # degrees per second - now configurable
      rate_limit = max_angle_rate * self.dt

      # Calculate desired change
      angle_diff = angle_steers_des - self._prev_angle_steers_des

      # Apply rate limiting
      angle_diff_limited = np.clip(angle_diff, -rate_limit, rate_limit)

      # Apply a smooth transition using a weighted average to avoid jerks
      # The smoothing factor is adaptive based on vehicle speed
      speed_factor = min(1.0, max(0.1, CS.vEgo / 15.0))  # More smoothing at higher speeds
      smoothing_factor = 0.3 * speed_factor  # Adjust between 0.03 (low speed) and 0.3 (high speed)

      # Apply smoothing
      target_desired = self._prev_angle_steers_des + angle_diff_limited
      angle_steers_des = (1.0 - smoothing_factor) * self._prev_angle_steers_des + smoothing_factor * target_desired
    self._prev_angle_steers_des = angle_steers_des

    error = angle_steers_des - CS.steeringAngleDeg

    pid_log.steeringAngleDesiredDeg = float(angle_steers_des)
    pid_log.angleError = float(error)
    if not active:
      output_torque = 0.0
      pid_log.active = False

    else:
      # offset does not contribute to resistive torque
      ff = self.ff_factor * self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)
      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

      # Enhanced adaptive PID parameters for even smoother response
      # Implement speed and curvature adaptive gains to reduce oscillations
      original_kp_values = self.pid._k_p[1]
      original_ki_values = self.pid._k_i[1]

      # Calculate adaptive gain reduction based on vehicle state
      speed_factor = min(1.0, max(0.1, CS.vEgo / 20.0))  # Normalize speed effect
      curvature_factor = min(1.0, abs(desired_curvature) * 30.0)  # Reduce gains with high curvature

      # More sophisticated gain adaptation
      if CS.vEgo > self.high_speed_threshold:  # Above configurable threshold (default 15 m/s or 54 km/h)
        # Reduce both P and I gains at high speeds to prevent oscillations
        temp_kp = min(self.pid.k_p, original_kp_values[0] * 0.8) if len(original_kp_values) > 0 else self.pid.k_p * 0.8
        temp_ki = min(self.pid.k_i, self.high_speed_ki_limit)  # Reduce ki to reduce oscillation - now configurable

        # Create new tuples for temporary gain values
        temp_kp_array = [temp_kp] * len(original_kp_values)
        temp_ki_array = [temp_ki] * len(original_ki_values)

        # Apply temporary gains
        self.pid._k_p = (self.pid._k_p[0], tuple(temp_kp_array))
        self.pid._k_i = (self.pid._k_i[0], tuple(temp_ki_array))

        # Add feedforward compensation at high speeds for better tracking
        ff *= (1.0 + curvature_factor * 0.2)  # Slight boost when tracking tight curves at high speed

        output_torque = self.pid.update(error,
                                  feedforward=ff,
                                  speed=CS.vEgo,
                                  freeze_integrator=freeze_integrator)

      else:
        # Speed-adaptive gains for low speeds
        gain_reduction = 1.0 - (1.0 - speed_factor) * 0.3  # Up to 30% gain at very low speeds
        temp_kp = self.pid.k_p * gain_reduction
        temp_ki = min(self.pid.k_i, original_ki_values[0] * gain_reduction) if len(original_ki_values) > 0 else self.pid.k_i * gain_reduction

        temp_kp_array = [temp_kp] * len(original_kp_values)
        temp_ki_array = [temp_ki] * len(original_ki_values)

        self.pid._k_p = (self.pid._k_p[0], tuple(temp_kp_array))
        self.pid._k_i = (self.pid._k_i[0], tuple(temp_ki_array))

        output_torque = self.pid.update(error,
                                  feedforward=ff,
                                  speed=CS.vEgo,
                                  freeze_integrator=freeze_integrator)

      # Restore original gains after update
      self._restore_original_pid_gains(original_kp_values, original_ki_values)

      # Add output rate limiting for even smoother actuation
      if hasattr(self, '_prev_output_torque'):
          max_torque_rate = min(2.0, max(0.5, CS.vEgo * 0.1))  # Adaptive torque rate limit
          torque_change_limit = max_torque_rate * self.dt
          torque_change = output_torque - self._prev_output_torque
          torque_change_limited = np.clip(torque_change, -torque_change_limit, torque_change_limit)
          output_torque = self._prev_output_torque + torque_change_limited
      self._prev_output_torque = output_torque

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_torque)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    return output_torque, angle_steers_des, pid_log
