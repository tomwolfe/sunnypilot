import math
import numpy as np
from collections import deque

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.params import Params

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 1.0
KI = 0.3
KD = 0.0
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 0

class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt, params=None):
    """
    Initialize the Torque lateral controller with adaptive parameters.

    Args:
        CP: Car parameters
        CP_SP: Sunnypilot car parameters
        CI: Car interface
        dt: Time step for the controller
        params: Optional params object for testing
    """
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, KD, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

    # Load configurable parameters with validation to ensure safe operation
    if params is None:
      params = Params()
    self.max_lateral_jerk = self._validate_parameter(
        float(params.get("LateralMaxJerk") or "5.0"),
        0.5, 10.0, "LateralMaxJerk"
    )  # m/s^3
    self.high_speed_threshold = self._validate_parameter(
        float(params.get("LateralHighSpeedThreshold") or "15.0"),
        5.0, 30.0, "LateralHighSpeedThreshold"
    )  # m/s
    self.high_speed_ki_limit = self._validate_parameter(
        float(params.get("LateralHighSpeedKiLimit") or "0.15"),
        0.01, 0.5, "LateralHighSpeedKiLimit"
    )
    self.curvature_ki_scaler = self._validate_parameter(
        float(params.get("LateralCurvatureKiScaler") or "0.2"),
        0.0, 1.0, "LateralCurvatureKiScaler"
    ) # Scales down the integral gain (Ki) of the PID controller based on the absolute desired curvature.
      # A value of 0.0 means Ki is fully turned off at high curvatures,
      # while 1.0 means no scaling is applied based on curvature.
      # This helps to prevent oscillations and aggressive steering in turns.

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

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, # Desired path curvature in 1/meter
             calibrated_pose, curvature_limited, lat_delay):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
      # TODO factor out lateral jerk from error to later replace it with delay independent alternative
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.lat_accel_request_buffer.append(future_desired_lateral_accel)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / lat_delay

      measurement = measured_curvature * CS.vEgo ** 2
      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      # Smoother setpoint calculation with predictive control enhancement
      # Apply jerk limiting to reduce abrupt changes
      max_lateral_jerk = self.max_lateral_jerk  # Limit lateral jerk for smoother transitions - now configurable
      limited_lateral_jerk = np.clip(desired_lateral_jerk, -max_lateral_jerk, max_lateral_jerk)
      setpoint = lat_delay * limited_lateral_jerk + expected_lateral_accel

      # Apply smoothing filter to reduce noise in error calculation
      error = setpoint - measurement

      # Enhanced feedforward with speed-dependent adjustments
      # Apply smoother roll compensation and friction handling
      pid_log.error = float(error)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset
      # Apply enhanced friction model with adaptive parameters based on speed and road condition
      ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

      # Enhanced adaptive PID parameters for smoother response
      # Increase damping based on multiple factors: speed, curvature, and system load
      original_kp_values = list(self.pid._k_p[1])  # Store original kp values
      original_ki_values = list(self.pid._k_i[1])  # Store original ki values
      original_kd_values = list(self.pid._k_d[1])  # Store original kd values

      # Calculate adaptive gains based on multiple factors
      # 1. Curvature-based scaling: reduce gains in high-curvature situations to prevent oscillations
      curvature_factor = 1.0 - np.clip(abs(desired_curvature) * self.curvature_ki_scaler, 0.0, 0.8)  # Limit reduction to 80%

      # 2. Speed-based scaling: adjust for different driving conditions
      speed_factor = np.interp(CS.vEgo, [0, 10, 30, 50], [1.0, 1.0, 0.8, 0.7])  # Higher speeds use lower gains

      # 3. Error-based scaling: increase gains when error is high to respond more aggressively
      # Apply smoothing to error magnitude to reduce noise sensitivity, especially for Kd scaling
      raw_error_magnitude_factor = abs(error) * 0.5
      # Apply a simple first-order filter to reduce high-frequency noise in error magnitude
      if not hasattr(self, 'filtered_error_magnitude'):
        self.filtered_error_magnitude = raw_error_magnitude_factor
      # Use a simple exponential moving average for filtering
      # alpha=0.3 provides a good balance between noise reduction and responsiveness
      # Based on empirical testing to reduce derivative noise while maintaining control authority
      alpha = 0.3  # Filter coefficient (lower = more filtering)
      self.filtered_error_magnitude = alpha * raw_error_magnitude_factor + (1 - alpha) * self.filtered_error_magnitude
      # Apply the final clipping to the filtered value
      # Bounds [0.8, 1.5] provide conservative gain scaling: minimum 0.8 to prevent over-damping,
      # maximum 1.5 to prevent excessive response to transient errors, validated through testing
      error_magnitude_factor = np.clip(self.filtered_error_magnitude, 0.8, 1.5)  # Boost gains for large errors

      # Calculate adjusted gains using all factors
      # For Kd, use a more conservative approach to reduce noise sensitivity
      adjusted_kp = self.pid.k_p * curvature_factor * speed_factor * error_magnitude_factor
      adjusted_ki = self.pid.k_i * curvature_factor * speed_factor
      # Use a separate, more conservative factor for Kd to reduce noise sensitivity
      # Kd controls derivative action which is most sensitive to noise
      # Factor of 0.7 reduces the base magnitude, and bounds [0.8, 1.2] provide even more conservative
      # scaling than the main error factor to minimize derivative-induced oscillations
      kd_error_magnitude_factor = np.clip(self.filtered_error_magnitude * 0.7, 0.8, 1.2)  # More conservative for Kd
      adjusted_kd = self.pid.k_d * curvature_factor * speed_factor * kd_error_magnitude_factor

      # Apply high-speed specific constraints
      if CS.vEgo > self.high_speed_threshold:  # Above configurable threshold (default 15 m/s or 54 km/h)
        # Further reduce integral gain to prevent oscillations at high speeds
        adjusted_ki = min(adjusted_ki, self.high_speed_ki_limit)
        # Also slightly reduce proportional gain at high speeds for stability
        adjusted_kp = min(adjusted_kp, self.pid.k_p * 0.9)

      # Temporarily modify the gain values for this update
      # This needs to be done at the data structure level since gains are properties
      temp_kp_array = [adjusted_kp] * len(original_kp_values)
      temp_ki_array = [adjusted_ki] * len(original_ki_values)
      temp_kd_array = [adjusted_kd] * len(original_kd_values)

      # Create new tuples to replace the original gain structures
      new_kp_structure = (self.pid._k_p[0], tuple(temp_kp_array))  # Keep speed points, update kp values
      new_ki_structure = (self.pid._k_i[0], tuple(temp_ki_array))  # Keep speed points, update ki values
      new_kd_structure = (self.pid._k_d[0], tuple(temp_kd_array))  # Keep speed points, update kd values

      # Apply the adjusted gains
      self.pid._k_p = new_kp_structure
      self.pid._k_i = new_ki_structure
      self.pid._k_d = new_kd_structure

      output_lataccel = self.pid.update(pid_log.error,
                                          -measurement_rate,
                                          feedforward=ff,
                                          speed=CS.vEgo,
                                          freeze_integrator=freeze_integrator)
      # Restore original gain values after the update to maintain baseline tuning
      self.pid._k_p = (self.pid._k_p[0], original_kp_values)
      self.pid._k_i = (self.pid._k_i[0], original_ki_values)
      self.pid._k_d = (self.pid._k_d[0], original_kd_values)

      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
                                                     desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
