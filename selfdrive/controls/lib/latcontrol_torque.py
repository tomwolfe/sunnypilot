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



  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, # Desired path curvature in 1/meter
             calibrated_pose, curvature_limited, lat_delay, adaptive_gains: Dict):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    if not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      # Apply adaptive gains from LightweightAdaptiveGainScheduler
      lateral_gains = adaptive_gains.get('lateral', {})
      self.pid.kp = lateral_gains.get('kp', self.pid.kp)
      self.pid.ki = lateral_gains.get('ki', self.pid.ki)
      self.pid.kd = lateral_gains.get('kd', self.pid.kd)
      self.pid.kf = lateral_gains.get('kf', self.pid.kf)

      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
      
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.lat_accel_request_buffer.append(future_desired_lateral_accel)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      
      # Setpoint is directly the future desired lateral acceleration
      setpoint = future_desired_lateral_accel

      measurement = measured_curvature * CS.vEgo ** 2
      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      error = setpoint - measurement

      pid_log.error = float(error)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset
      # Apply enhanced friction model with adaptive parameters based on speed and road condition
      ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5

      output_lataccel = self.pid.update(pid_log.error,
                                          -measurement_rate,
                                          feedforward=ff,
                                          speed=CS.vEgo,
                                          freeze_integrator=freeze_integrator)
      
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
      pid_log.desiredLateralJerk = float(0.0) # Adaptive jerk removed, set to 0.0

      # Saturation logic: when NNLC is enabled, also consider physical torque saturation
      # Otherwise, only consider curvature limited state
      if (hasattr(self.extension, '_nnlc_enabled') and
          getattr(self.extension, '_nnlc_enabled', False)):
        # When NNLC is enabled, saturation occurs when path is curvature limited
        # or when the output torque is at its limits
        pid_log.saturated = bool(self._check_saturation(
          curvature_limited or abs(output_torque) >= self.steer_max - 1e-3,
          CS, steer_limited_by_safety, curvature_limited))
      else:
        # Standard behavior: only based on curvature limited
        pid_log.saturated = bool(self._check_saturation(curvature_limited, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log

  def update_thermal_compensation(self, thermal_stress_level, compensation_factor):
    """
    Update controller parameters based on thermal stress level to maintain
    consistent performance under different thermal conditions.

    Args:
      thermal_stress_level: 0=normal, 1=moderate, 2=high, 3=very high
      compensation_factor: Factor (0.0-1.0) indicating overall system compensation needed
    """
    # Store thermal parameters for use in the update method
    self.thermal_stress_level = thermal_stress_level
    self.thermal_performance_factor = compensation_factor

    # Adjust PID parameters based on thermal stress to maintain consistent control
    if thermal_stress_level >= 2:  # High or very high thermal stress
      # Reduce integral gain to avoid windup during thermal throttling
      self.pid.k_i = min(self.pid.k_i, self.torque_params.friction / 2.0)
      # Slightly reduce proportional gain to avoid oscillations during thermal stress
      self.pid.k_p = min(self.pid.k_p, self.pid.k_p * 0.9)
    elif thermal_stress_level == 1:  # Moderate thermal stress
      # Mild adjustment for moderate stress
      self.pid.k_i = min(self.pid.k_i, self.torque_params.friction * 0.8)

    # Apply compensation factor to prevent integral windup during throttling
    if hasattr(self, '_prev_integral') and compensation_factor < 0.8:
      # Reduce the integral term when system is under high thermal stress
      self.pid.error_integral *= compensation_factor

  def update_thermal_params(self, thermal_stress_level, performance_compensation_factor):
    """
    Update the thermal parameters used in adaptive gain calculation.
    This is called from the main control loop to pass thermal information to the lateral controller.

    Args:
      thermal_stress_level: 0=normal, 1=moderate, 2=high, 3=very high
      performance_compensation_factor: Factor (0.0-1.0) indicating overall system performance capacity
    """
    self.thermal_stress_level = thermal_stress_level
    self.thermal_performance_factor = performance_compensation_factor
