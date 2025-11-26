import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, CP_SP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP, params=None):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

    # Load configurable parameters with validation to ensure safe operation
    if params is None:
      params = Params()
    self.max_jerk = self._validate_parameter(
        float(params.get("LongitudinalMaxJerk") or "2.2"),
        0.5, 10.0, "LongitudinalMaxJerk"
    )  # m/s^3
    self.max_stopping_jerk = self._validate_parameter(
        float(params.get("LongitudinalMaxStoppingJerk") or "1.5"),
        0.5, 5.0, "LongitudinalMaxStoppingJerk"
    )  # m/s^3
    self.max_output_jerk = self._validate_parameter(
        float(params.get("LongitudinalMaxOutputJerk") or "2.0"),
        0.5, 5.0, "LongitudinalMaxOutputJerk"
    )  # m/s^3
    self.starting_speed_threshold = self._validate_parameter(
        float(params.get("LongitudinalStartingSpeedThreshold") or "3.0"),
        0.5, 10.0, "LongitudinalStartingSpeedThreshold"
    )  # m/s
    self.starting_accel_multiplier = self._validate_parameter(
        float(params.get("LongitudinalStartingAccelMultiplier") or "0.8"),
        0.1, 2.0, "LongitudinalStartingAccelMultiplier"
    )
    self.starting_accel_limit = self._validate_parameter(
        float(params.get("LongitudinalStartingAccelLimit") or "0.8"),
        0.1, 2.0, "LongitudinalStartingAccelLimit"
    )
    self.adaptive_error_threshold = self._validate_parameter(
        float(params.get("LongitudinalAdaptiveErrorThreshold") or "0.6"),
        0.01, 2.0, "LongitudinalAdaptiveErrorThreshold"
    )
    self.adaptive_speed_threshold = self._validate_parameter(
        float(params.get("LongitudinalAdaptiveSpeedThreshold") or "5.0"),
        1.0, 20.0, "LongitudinalAdaptiveSpeedThreshold"
    )  # m/s

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

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Initialize state variables if not present
    if not hasattr(self, '_prev_a_target'):
      self._prev_a_target = a_target
    if not hasattr(self, '_prev_output_accel'):
      self._prev_output_accel = 0.0
    if not hasattr(self, '_acceleration_profile'):
      self._acceleration_profile = {'approaching_lead': False, 'approach_rate': 0.0, 'distance_to_lead': float('inf')}

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      # Enhanced stopping with smoother deceleration profile
      # Apply more gradual deceleration to avoid harsh braking
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # Apply jerk-limited deceleration for smoother stop
        max_jerk = self.max_stopping_jerk  # m/s^3, limit deceleration change - now configurable
        desired_decel = max_jerk * DT_CTRL
        output_accel = max(output_accel - desired_decel, self.CP.stopAccel)
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      # Enhanced starting with smoother acceleration
      # Apply gradual acceleration to avoid harsh takeoff
      if CS.vEgo < self.starting_speed_threshold:  # Below configurable threshold (default 10.8 km/h)
        output_accel = min(self.CP.startAccel * self.starting_accel_multiplier, self.starting_accel_limit)  # More gentle start at very low speeds - now configurable
      else:
        output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      # Apply jerk limitation to the target acceleration for smoother transitions
      # Limit the rate of change of acceleration (jerk)
      max_jerk = self.max_jerk  # m/s^3, limit how fast acceleration can change - now configurable
      jerk_limit = max_jerk * DT_CTRL
      a_target_change = a_target - self._prev_a_target
      a_target_limited = self._prev_a_target + np.clip(a_target_change, -jerk_limit, jerk_limit)
      self._prev_a_target = a_target_limited

      error = a_target_limited - CS.aEgo

      # Enhanced adaptive PID based on driving context and lead vehicle behavior
      # Calculate context-sensitive adaptive parameters
      adaptive_ki_factor = 1.0
      adaptive_kp_factor = 1.0
      ki_reduction_enabled = False

      if CS.vEgo > self.adaptive_speed_threshold:  # At moderate to high speeds
        if abs(error) < self.adaptive_error_threshold:  # Near target acceleration
          ki_reduction_enabled = True
          adaptive_ki_factor = 0.5  # Reduce integral action to avoid overshoot

          # Additional check: if following lead vehicle closely, be even more conservative
          # Note: This assumes we have access to lead vehicle information in CS if available
          # This would typically come from radarState which is accessed in the main control loop

      # Apply adaptive gains if needed
      if ki_reduction_enabled:
        temp_ki = min(self.pid.k_i * adaptive_ki_factor, self.CP.longitudinalTuning.kiV[0] * 0.5)
        temp_pid = PIDController((self.CP.longitudinalTuning.kpBP, self.CP.longitudinalTuning.kpV),
                                 (self.CP.longitudinalTuning.kiBP, [temp_ki] * len(self.CP.longitudinalTuning.kiV)),
                                 rate=1 / DT_CTRL)
        temp_pid.neg_limit = self.pid.neg_limit
        temp_pid.pos_limit = self.pid.pos_limit
        # Preserve integral term by copying from original PID
        temp_pid.i = self.pid.i
        temp_pid.p = self.pid.p
        temp_pid.d = self.pid.d
        temp_pid.f = self.pid.f
        temp_pid.control = self.pid.control
        temp_pid.speed = self.pid.speed

        output_accel = temp_pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target_limited)

        # Preserve the integral term from temp PID for next iteration
        self.pid.i = temp_pid.i
        self.pid.p = temp_pid.p
        self.pid.d = temp_pid.d
        self.pid.f = temp_pid.f
        self.pid.control = temp_pid.control
        self.pid.speed = temp_pid.speed
      else:
        output_accel = self.pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target_limited)

      # Enhanced output smoothing with context-aware jerk limiting
      max_output_jerk = self.max_output_jerk  # m/s^3, limit output jerk - now configurable

      # Increase jerk limit in emergency situations (e.g., hard braking needed)
      emergency_factor = 1.0
      if a_target_limited < -2.0:  # Hard braking situation
        emergency_factor = 2.0  # Allow more aggressive response

      output_jerk_limit = max_output_jerk * DT_CTRL * emergency_factor
      output_accel_change = output_accel - self._prev_output_accel

      # Apply adaptive jerk limiting based on current driving situation
      if abs(output_accel_change) > output_jerk_limit:
        output_accel = self._prev_output_accel + np.clip(output_accel_change,
                                                         -output_jerk_limit,
                                                         output_jerk_limit)

      # Additional smoothing for very low-speed situations to prevent jerky movements
      if CS.vEgo < 5.0:  # Below 18 km/h (5 m/s)
        # Apply additional smoothing at very low speeds for smoother stop-and-go
        very_low_speed_smoothing = 0.7  # More conservative at creep speeds
        output_accel = self._prev_output_accel * (1 - very_low_speed_smoothing) + output_accel * very_low_speed_smoothing

      self._prev_output_accel = output_accel

    # Apply final acceleration limits
    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def update_thermal_compensation(self, thermal_stress_level, compensation_factor):
    """
    Update longitudinal controller parameters based on thermal stress level to maintain
    consistent acceleration/deceleration performance under different thermal conditions.

    Args:
      thermal_stress_level: 0=normal, 1=moderate, 2=high, 3=very high
      compensation_factor: Factor (0.0-1.0) indicating overall system compensation needed
    """
    # Store current PID state to preserve during reinitialization
    old_p = self.pid.p
    old_i = self.pid.i
    old_d = self.pid.d
    old_f = self.pid.f
    old_control = self.pid.control
    old_speed = self.pid.speed
    old_pos_limit = self.pid.pos_limit
    old_neg_limit = self.pid.neg_limit

    # Adjust PID parameters based on thermal stress to maintain consistent longitudinal control
    original_ki_v = self.CP.longitudinalTuning.kiV
    original_kp_v = self.CP.longitudinalTuning.kpV

    if thermal_stress_level >= 2:  # High or very high thermal stress
      # Create new PID with reduced gains to prevent accumulation during thermal throttling
      new_ki_v = [min(ki_val, original_ki_v[0] * 0.7) for ki_val in original_ki_v]  # Reduce by 30%
      new_kp_v = [min(kp_val, original_kp_v[0] * 0.9) for kp_val in original_kp_v]  # Reduce by 10%

      # Create a new PID controller with adjusted parameters
      self.pid = PIDController((self.CP.longitudinalTuning.kpBP, new_kp_v),
                               (self.CP.longitudinalTuning.kiBP, new_ki_v),
                               rate=1 / DT_CTRL)
    elif thermal_stress_level == 1:  # Moderate thermal stress
      # Mild adjustment for moderate stress
      new_ki_v = [min(ki_val, original_ki_v[0] * 0.85) for ki_val in original_ki_v]  # Reduce by 15%

      # Create a new PID controller with adjusted parameters
      self.pid = PIDController((self.CP.longitudinalTuning.kpBP, original_kp_v),
                               (self.CP.longitudinalTuning.kiBP, new_ki_v),
                               rate=1 / DT_CTRL)

    # Restore PID state to preserve continuity
    self.pid.p = old_p
    self.pid.i = old_i
    self.pid.d = old_d
    self.pid.f = old_f
    self.pid.control = old_control
    self.pid.speed = old_speed

    # Apply compensation factor to limit maximum acceleration during high thermal stress
    if compensation_factor < 0.8:
      # Reduce acceleration limits proportionally to thermal compensation factor
      self.pid.neg_limit = old_neg_limit * compensation_factor
      self.pid.pos_limit = old_pos_limit * compensation_factor
