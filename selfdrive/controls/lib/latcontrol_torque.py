import math
import numpy as np
from collections import deque

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

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

KP = 1.8
KI = 0.5
KD = 0.0
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]
# Define the curvature gain interpolation.
# The first array represents absolute curvature values (in 1/meter).
# The second array represents the corresponding gain multipliers for the proportional term.
# Values outside the defined curvature range will be clamped to the nearest boundary.
#
# Default values explanation:
# - Curvature 0.06 m^-1 corresponds to a 16.7m radius turn, typical for sharp urban corners.
# - A gain multiplier of 2.0 means proportional gain is doubled for the sharpest turns.
#
# Tuning guidance:
# These values can be tuned for different vehicle types based on steering ratio and tire characteristics.
# Increasing the gain multiplier will make the steering more aggressive in turns.
# Ensure that the curvature values are non-negative and in ascending order, and gain multipliers are >= 1.0.
CURVATURE_GAIN_INTERP = [[0.0, 0.02, 0.04, 0.06], [1.0, 1.2, 1.5, 2.0]]

LP_FILTER_CUTOFF_HZ = 1.2
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 0

class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    k_curvature = CP_SP.curvatureGainInterp if CP_SP.curvatureGainInterp else CURVATURE_GAIN_INTERP

    # Initialize PID controller with original parameters
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, KD, rate=1/self.dt, k_curvature=k_curvature)

    # Initialize adaptive control components
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    # Adaptive control parameters
    self.base_gains = {'kp': KP, 'ki': KI, 'kd': KD}
    self.previous_curvature = 0.0
    self.steering_effort_history = deque(maxlen=20)  # Store recent steering efforts
    self.lateral_accel_history = deque(maxlen=20)    # Store recent lateral accelerations

    # Road condition estimation
    self.road_condition = 'normal'  # 'normal', 'icy', 'wet', 'rough'

    # Environmental condition estimator
    self.environmental_condition = 'normal'  # To track environmental conditions from external sources

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def estimate_environmental_conditions(self, CS, params, sm):
    """Estimate environmental conditions based on car state and sensor data"""
    # This method would integrate with the safety monitoring system
    # For now, we'll use a placeholder that could be updated with actual data from the safety monitor

    # Check for environmental conditions from external sources if available
    if hasattr(sm, 'safety_monitor_state') and sm.safety_monitor_state is not None:
      # Use safety monitor report to determine environmental conditions
      if hasattr(sm.safety_monitor_state, 'road_condition'):
        self.road_condition = sm.safety_monitor_state.get('road_condition', 'normal')
      if hasattr(sm.safety_monitor_state, 'weather_condition'):
        self.environmental_condition = sm.safety_monitor_state.get('weather_condition', 'normal')

    # Also estimate based on car behavior
    if CS.vEgo > 5.0 and abs(CS.aEgo) > 0.5:  # Vehicle is moving and accelerating/decelerating
      # Check for signs of slippery conditions based on steering vs lateral acceleration
      # This is a simplified approach - real implementation would be more sophisticated
      pass

    return self.road_condition, self.environmental_condition

  def calculate_adaptive_gains(self, v_ego, curvature, environmental_conditions):
    """Calculate adaptive PID gains based on speed, curvature, and environmental conditions"""
    # Base adaptive factors
    speed_factor = np.interp(v_ego, [5, 15, 30], [1.5, 1.0, 0.8])  # Higher gains at low speeds
    curvature_factor = min(1.5, 1.0 + abs(curvature) * 100)  # Higher gains for higher curvature

    # Environmental condition adjustments
    condition_factor = 1.0

    if environmental_conditions.get('road_condition', 'normal') in ['icy', 'wet']:
      condition_factor = 0.6  # Reduce gains in slippery conditions
    elif environmental_conditions.get('road_condition', 'normal') == 'rough':
      condition_factor = 0.8  # Moderate reduction for bumpy roads
    elif environmental_conditions.get('weather_condition', 'normal') in ['rain', 'snow', 'fog']:
      condition_factor = 0.75  # Reduce gains in poor weather

    # Calculate adaptive gains
    kp = self.base_gains['kp'] * speed_factor * curvature_factor * condition_factor
    ki = self.base_gains['ki'] * speed_factor * curvature_factor * condition_factor * 0.7  # Reduce integral action in challenging conditions
    kd = self.base_gains['kd'] * speed_factor * curvature_factor * condition_factor

    return kp, ki, kd

  def update_pid_gains(self, new_kp, new_ki, new_kd):
    """Update PID controller gains"""
    self.pid.k_p = new_kp
    self.pid.k_i = new_ki
    self.pid.k_d = new_kd

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay, sm=None):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    # Estimate environmental conditions
    env_conditions = {}
    if sm is not None:
      road_condition, weather_condition = self.estimate_environmental_conditions(CS, params, sm)
      env_conditions = {'road_condition': road_condition, 'weather_condition': weather_condition}
    else:
      env_conditions = {'road_condition': 'normal', 'weather_condition': 'normal'}

    # Calculate adaptive gains based on current conditions and update PID
    adaptive_kp, adaptive_ki, adaptive_kd = self.calculate_adaptive_gains(CS.vEgo, desired_curvature, env_conditions)
    self.update_pid_gains(adaptive_kp, adaptive_ki, adaptive_kd)

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # Add safety check for desired_curvature
      if desired_curvature is None or not np.isfinite(desired_curvature):
        desired_curvature = 0.0
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
      # TODO factor out lateral jerk from error to later replace it with delay independent alternative
      # desired_curvature is the desired curvature from the path planner (e.g., modeld)
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.lat_accel_request_buffer.append(future_desired_lateral_accel)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / lat_delay

      measurement = measured_curvature * CS.vEgo ** 2
      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
      error = setpoint - measurement

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset
      # TODO jerk is weighted by lat_delay for legacy reasons, but should be made independent of it
      ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                       -measurement_rate,
                                        feedforward=ff,
                                        speed=CS.vEgo,
                                        curvature=desired_curvature,
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
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

      # Store recent values for adaptive control
      if active:
        self.steering_effort_history.append(abs(output_torque))
        self.lateral_accel_history.append(abs(measurement))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
