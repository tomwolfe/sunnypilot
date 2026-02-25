import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_calibrator import LongitudinalCalibrator

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
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.calibrator = LongitudinalCalibrator()

  def reset(self):
    self.pid.reset()
    self.calibrator.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, accel_neural=0.0, longitudinal_uncertainty=0.0):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    
    # Neural Longitudinal Override:
    # If the model is certain (low xStd), we blend its predicted acceleration directly.
    # This allows for "smooth, human-like" braking that a simple PID often struggles with.
    use_neural = accel_neural != 0.0 and self.long_control_state in [LongCtrlState.starting, LongCtrlState.pid]
    
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      # Calculate PID output as fallback
      error = a_target - CS.aEgo
      pid_accel = self.pid.update(error, speed=CS.vEgo, feedforward=a_target)
      
      if use_neural:
        # Bayesian Longitudinal Blend:
        # Scale neural authority based on uncertainty (xStd proxy)
        # 0.5m uncertainty is "perfect", 3.0m is "noisy"
        
        # Online Calibration Offset:
        # Compare model's predicted acceleration to actual aEgo over a window.
        # If the model is consistently wrong, we increase the effective uncertainty.
        uncertainty_offset = self.calibrator.update(accel_neural, CS.aEgo, use_neural)
        effective_uncertainty = longitudinal_uncertainty + uncertainty_offset
        
        # Dynamic Sigmoid Center: lower center at higher speeds for more caution
        # At 0 m/s: center = 1.5m
        # At 30 m/s: center = 1.0m
        sigmoid_center = float(np.interp(CS.vEgo, [0, 30], [1.5, 1.0]))
        sigmoid_steepness = 4.0
        
        w_neural = 1.0 / (1.0 + np.exp(sigmoid_steepness * (effective_uncertainty - sigmoid_center)))
        
        # In 'Blended' (E2E) mode, we prioritize the model's intent
        output_accel = w_neural * accel_neural + (1.0 - w_neural) * pid_accel
      else:
        output_accel = pid_accel
        # Keep calibrator updated even when not actively using neural for better transition
        self.calibrator.update(accel_neural, CS.aEgo, False)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
