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

    pid_log.steeringAngleDesiredDeg = float(angle_steers_des)
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
      original_ki_values = self.pid._k_i[1]  # Store original ki values (tuple)
      if CS.vEgo > self.high_speed_threshold:  # Above configurable threshold (default 15 m/s or 54 km/h)
        # Temporarily reduce integral gain to prevent oscillations at high speeds
        # This is more robust than creating temporary PID controllers
        temp_ki = min(self.pid.k_i, self.high_speed_ki_limit)  # Reduce ki to reduce oscillation - now configurable
        # We can't directly modify the property of PIDController, so we need to update the internal values temporarily
        # Create new array that maintains the same length as original, with all values set to temp_ki
        temp_ki_array = [temp_ki] * len(original_ki_values)
        # Create new tuple to replace the original
        new_ki_structure = (self.pid._k_i[0], tuple(temp_ki_array))  # Keep speed points, update ki values
        self.pid._k_i = new_ki_structure
        output_torque = self.pid.update(error,
                                  feedforward=ff,
                                  speed=CS.vEgo,
                                  freeze_integrator=freeze_integrator)
        # Restore original ki values after update
        self.pid._k_i = (self.pid._k_i[0], original_ki_values)
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
