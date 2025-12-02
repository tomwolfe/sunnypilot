import numpy as np
from typing import Dict

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

    self.params = params

    def _get_param_value(key, default, converter=float):
      if self.params and self.params.get(key) is not None:
        try:
          return converter(self.params.get(key).decode('utf8'))
        except (ValueError, AttributeError):
          pass
      return default

    self.max_jerk = _get_param_value("LongitudinalMaxJerk", 2.2)
    self.max_stopping_jerk = _get_param_value("LongitudinalMaxStoppingJerk", 1.5)
    self.max_output_jerk = _get_param_value("LongitudinalMaxOutputJerk", 2.0)
    self.starting_speed_threshold = _get_param_value("LongitudinalStartingSpeedThreshold", 3.0)
    self.starting_accel_multiplier = _get_param_value("LongitudinalStartingAccelMultiplier", 0.8)
    self.starting_accel_limit = _get_param_value("LongitudinalStartingAccelLimit", 0.8)
    self.adaptive_error_threshold = _get_param_value("LongitudinalAdaptiveErrorThreshold", 0.6)
    self.adaptive_speed_threshold = _get_param_value("LongitudinalAdaptiveSpeedThreshold", 5.0)


  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, adaptive_gains: Dict):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Apply adaptive gains from LightweightAdaptiveGainScheduler
    long_gains = adaptive_gains.get('longitudinal', {})

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.
      self.last_output_accel = output_accel
      return output_accel

    elif self.long_control_state == LongCtrlState.stopping:
      # Enhanced stopping with smoother deceleration profile
      # Apply more gradual deceleration to avoid harsh braking
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # Apply jerk-limited deceleration for smoother stop
        max_jerk = 1.5  # m/s^3, limit deceleration change - now fixed or handled by LightweightComfortOptimizer
        desired_decel = max_jerk * DT_CTRL
        output_accel = max(output_accel - desired_decel, self.CP.stopAccel)
      self.reset()
      self.last_output_accel = output_accel
      return output_accel

    elif self.long_control_state == LongCtrlState.starting:
      # Enhanced starting with smoother acceleration
      # Apply gradual acceleration to avoid harsh takeoff
      if CS.vEgo < 3.0:  # Below 3.0 m/s (default 10.8 km/h)
        base_accel = self.CP.startAccel * 0.8
        output_accel = min(base_accel, 0.8)  # More gentle start at very low speeds
      else:
        output_accel = self.CP.startAccel
      self.reset()
      self.last_output_accel = output_accel
      return output_accel

    else:  # LongCtrlState.pid
      # error is the difference between target and current acceleration
      error = a_target - CS.aEgo
      # Apply kf from adaptive gains to feedforward
      kf_factor = long_gains.get('kf', 1.0) # Default to 1.0 if not found
      feedforward_accel = a_target * kf_factor
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=feedforward_accel,
                                     k_p_override=long_gains.get('kp'),
                                     k_i_override=long_gains.get('ki'))

      self.last_output_accel = output_accel
      return output_accel

  def _calculate_adaptive_output_jerk_limit(self, CS, a_target_limited, current_output_accel):
    """
    Calculate adaptive output jerk limits based on driving context and stability.

    Critical Analysis Note: This function ensures that the system's acceleration output
    remains smooth and safe, dynamically adjusting based on various factors. This is
    fundamental for comfort and safety. Logging the dynamic adjustments to the
    jerk limit and their impact on `output_accel` is crucial for understanding
    system behavior under different driving conditions and for future tuning.

    Args:
        CS: CarState object
        a_target_limited: Limited target acceleration after jerk limiting
        current_output_accel: Current acceleration output before limiting

    Returns:
        float: Adaptive output jerk limit in m/s^3
    """
    # Start with configurable base limit
    base_output_jerk_limit = self.max_output_jerk

    # Adjust based on difference between target and current acceleration
    accel_error = abs(a_target_limited - current_output_accel)
    if accel_error > 2.0:  # Large error - allow more aggressive correction
        base_output_jerk_limit *= 1.2
    elif accel_error < 0.5:  # Small error - be more conservative
        base_output_jerk_limit *= 0.8

    # Reduce jerk when following lead vehicle closely
    if hasattr(CS, 'radarState') and CS.radarState is not None:
        try:
            if hasattr(CS.radarState, 'leadOne') and CS.radarState.leadOne.status:
                lead_one = CS.radarState.leadOne
                dRel = getattr(lead_one, 'dRel', None)

                # Only proceed if dRel is an actual number
                if isinstance(dRel, (int, float)):
                    if dRel < 30.0:
                        base_output_jerk_limit *= 0.85  # More conservative when close to lead
                        if dRel < 15.0:  # Very close
                            base_output_jerk_limit *= 0.7  # Much more conservative
        except (AttributeError, TypeError):
            pass

    # Reduce jerk at higher speeds for stability
    if CS.vEgo > 25.0:  # Above ~90 km/h
        base_output_jerk_limit *= 0.9

    # Further reduce jerk if vehicle is already accelerating rapidly
    if abs(current_output_accel) > 2.0:
        base_output_jerk_limit *= 0.85  # Be more conservative when already accelerating hard

    # Apply limits to ensure safety
    base_output_jerk_limit = max(0.5, min(8.0, base_output_jerk_limit))  # Reasonable bounds

    return base_output_jerk_limit


