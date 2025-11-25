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
      if hasattr(self, '_prev_a_target'):
        max_jerk = self.max_jerk  # m/s^3, limit how fast acceleration can change - now configurable
        jerk_limit = max_jerk * DT_CTRL
        a_target_change = a_target - self._prev_a_target
        a_target_limited = self._prev_a_target + np.clip(a_target_change, -jerk_limit, jerk_limit)
      else:
        a_target_limited = a_target
      self._prev_a_target = a_target_limited

      error = a_target_limited - CS.aEgo

      # Adaptive PID based on driving conditions
      # Reduce integral gain when close to target to avoid overshoot
      if abs(error) < self.adaptive_error_threshold and CS.vEgo > self.adaptive_speed_threshold:  # Near target at moderate speeds - now configurable
        # Create temporary PID with reduced integral gain
        temp_ki = min(self.pid.ki, self.CP.longitudinalTuning.kiV[0] * 0.5)  # Reduce integral action
        temp_pid = PIDController((self.CP.longitudinalTuning.kpBP, self.CP.longitudinalTuning.kpV),
                                 (self.CP.longitudinalTuning.kiBP, [temp_ki] * len(self.CP.longitudinalTuning.kiV)),
                                 rate=1 / DT_CTRL)
        temp_pid.neg_limit = self.pid.neg_limit
        temp_pid.pos_limit = self.pid.pos_limit
        temp_pid.error_integral = self.pid.error_integral  # Preserve integral term

        output_accel = temp_pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target_limited)

        # Preserve the original PID's integral term for next iteration
        self.pid.error_integral = temp_pid.error_integral
      else:
        output_accel = self.pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target_limited)

      # Apply additional smoothing to the output acceleration
      # Limit the rate of change of acceleration for comfort
      if hasattr(self, '_prev_output_accel'):
        max_output_jerk = self.max_output_jerk  # m/s^3, limit output jerk - now configurable
        output_jerk_limit = max_output_jerk * DT_CTRL
        output_accel_change = output_accel - self._prev_output_accel
        output_accel = self._prev_output_accel + np.clip(output_accel_change, -output_jerk_limit, output_jerk_limit)
      self._prev_output_accel = output_accel

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
