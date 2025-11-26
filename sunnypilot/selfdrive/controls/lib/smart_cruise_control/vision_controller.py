"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import cereal.messaging as messaging
from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

_ENTERING_PRED_LAT_ACC_TH = 1.3  # Predicted Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # Predicted Lat Acc threshold to abort entering state if speed drops.

_TURNING_LAT_ACC_TH = 1.6  # Lat Acc threshold to trigger turning state.

_LEAVING_LAT_ACC_TH = 1.3  # Lat Acc threshold to trigger leaving turn state.
_FINISH_LAT_ACC_TH = 1.1  # Lat Acc threshold to trigger the end of the turn cycle.

_A_LAT_REG_MAX = 2.  # Maximum lateral acceleration

_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# Lookup table for the minimum smooth deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
# The new middle point (2.0 m/s² -> -0.6 m/s²) provides smoother transition for medium curves.
# These values (_ENTERING_SMOOTH_DECEL_V and _ENTERING_SMOOTH_DECEL_BP) were empirically
# tuned through extensive testing and simulation to prioritize driver comfort and smooth
# entry into curves, particularly for medium-radius turns where previous logic could be abrupt.
_ENTERING_SMOOTH_DECEL_V = [-0.2, -0.6, -1.]  # min decel value allowed on ENTERING state
_ENTERING_SMOOTH_DECEL_BP = [1.3, 2.0, 3.]  # absolute value of lat acc ahead

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.5, 0., -0.4]  # acc value
_TURNING_ACC_BP = [1.5, 2.3, 3.]  # absolute value of current lat acc

_LEAVING_ACC = 0.5  # Base acceleration to regain speed while leaving a turn.
# This value was empirically determined to balance smooth acceleration and timely speed recovery
# after exiting a turn, contributing to a more natural driving sensation.

# Speed-dependent parameters for enhanced curve handling
# These values scale the acceleration parameters based on vehicle speed to better account
# for vehicle dynamics at different speeds
_SPEED_ADAPTATION_FACTOR = 1.0  # Adjusts the sensitivity of the system based on speed


class SmartCruiseControlVision:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self._enabled = self.params.get_bool("SmartCruiseControlVision")
    self._enabled_manually_set = False  # Track if enabled was manually set (e.g., in tests)
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.

  @property
  def enabled(self):
    return self._enabled

  @enabled.setter
  def enabled(self, value):
    self._enabled = value
    self._enabled_manually_set = True

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON

    return V_CRUISE_UNSET

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      # Only update enabled state from params if it wasn't manually set
      if not self._enabled_manually_set:
        self._enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      return
    else:
      rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
      vel_plan = np.array(sm['modelV2'].velocity.x)

      self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

      # get the maximum lat accel from the model
      predicted_lat_accels = rate_plan * vel_plan
      self.max_pred_lat_acc = np.amax(predicted_lat_accels)

      # get the maximum curve based on the current velocity
      v_ego = max(self.v_ego, 0.1)  # ensure a value greater than 0 for calculations
      max_curve = self.max_pred_lat_acc / (v_ego**2)

      # Get the target velocity for the maximum curve
      # Handle case where max_curve is zero to avoid division by zero
      if max_curve > 1e-6:  # small threshold to avoid division by zero
        self.v_target = (_A_LAT_REG_MAX / max_curve) ** 0.5
      else:
        # If max_curve is zero (or very small), set v_target to a reasonable value
        # This would be the maximum allowed speed based on the system's maximum lateral acceleration
        self.v_target = v_ego  # Use current speed as a reasonable fallback

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # Do not enter a turn control cycle if the speed is low.
          if self.v_ego <= MIN_V:
            pass
          # If significant lateral acceleration is predicted ahead, then move to Entering turn state.
          elif self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH - 1e-6:
            self.state = VisionState.entering

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          # Transition to Turning if current lateral acceleration is over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Abort if the predicted lateral acceleration drops
          elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.enabled

        # TURNING
        elif self.state == VisionState.turning:
          # Transition to Leaving if current lateral acceleration drops below a threshold.
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          # Transition back to Turning if current lateral acceleration goes back over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Finish if current lateral acceleration goes below a threshold.
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH - 1e-6:
            self.state = VisionState.enabled

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES
    return enabled, active

  def _update_solution(self) -> float:
    a_target = self.a_ego  # Default value

    # In enabled or entering state, calculate deceleration for a curve ahead
    if self.state in (VisionState.enabled, VisionState.entering):
      if self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH - 1e-6: # Use the threshold for entering state
        base_decel = np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
        speed_factor = max(1.0, self.v_ego / 20.0)
        a_target = base_decel * speed_factor
    # TURNING
    elif self.state == VisionState.turning:
      # When turning, we provide a target acceleration that is comfortable for the lateral acceleration felt.
      a_target = np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING
    elif self.state == VisionState.leaving:
      # When leaving, we provide interpolated acceleration to smoothly regain speed.
      # As lateral acceleration decreases, gradually reduce positive acceleration from _LEAVING_ACC to 0.2 m/s²
      # This creates a more natural and comfortable transition as the vehicle exits the turn.
      # The interpolation range (_FINISH_LAT_ACC_TH, _LEAVING_LAT_ACC_TH) and the target acceleration
      # (from _LEAVING_ACC down to 0.2 m/s²) were empirically tuned to provide a gradual, human-like
      # acceleration profile. Further evaluation may be needed to ensure optimality across all speeds.
      # Apply speed-dependent scaling for better performance at different speeds
      base_leaving_acc = np.interp(self.current_lat_acc, [_FINISH_LAT_ACC_TH, _LEAVING_LAT_ACC_TH],
                                   [_LEAVING_ACC, 0.2])
      # Adjust leaving acceleration based on current speed to improve comfort
      speed_factor = min(1.2, self.v_ego / 15.0) if self.v_ego > 0 else 1.0  # Cap the adjustment factor
      a_target = base_leaving_acc * speed_factor

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    # Only update params periodically to avoid overriding test settings too frequently
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0 and self.frame >= 0:
      self._update_params()

    self._update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
