"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from typing import TYPE_CHECKING

from cereal import log, custom

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.common.params import Params
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake, read_steering_mode_param, MADS_NO_ACC_MAIN_BUTTON
from openpilot.sunnypilot.mads.state import StateMachine, GEARS_ALLOW_PAUSED_SILENT

if TYPE_CHECKING:
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
SafetyModel = structs.CarParams.SafetyModel

SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)


class ModularAssistiveDrivingSystem:
  """
  The Modular Assistive Driving System (MADS) provides enhanced engagement/disengagement
  logic for sunnypilot, with features like:
  - Advanced brake handling (disengage vs pause modes)
  - Unified engagement mode
  - Silent event handling for smoother operation
  - Vehicle-specific behavior customization
  """

  def __init__(self, selfdrive: 'SelfdriveD'):
    """
    Initialize the Modular Assistive Driving System.

    Args:
      selfdrive: Reference to the main selfdrived instance
    """
    self.CP: structs.CarParams = selfdrive.CP
    self.CP_SP: custom.CarParamsSP = selfdrive.CP_SP
    self.params: Params = selfdrive.params

    # MADS state variables
    self.enabled: bool = False
    self.active: bool = False
    self.available: bool = False
    self.allow_always: bool = False  # Allow MADS engagement regardless of cruise state for certain vehicles
    self.no_main_cruise: bool = False  # Vehicle doesn't support standard cruise control main button
    self.selfdrive: SelfdriveD = selfdrive
    self.enabled_prev: bool = False  # Track previous enabled state within MADS
    self.state_machine: StateMachine = StateMachine(self)
    self.events: log.OnroadEvents = self.selfdrive.events
    self.events_sp: custom.OnroadEventsSP = self.selfdrive.events_sp
    self.disengage_on_accelerator: bool = Params().get_bool("DisengageOnAccelerator")

    # Vehicle-specific configurations
    if self.CP.brand == "hyundai":
      if self.CP.flags & (HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.CANFD):
        self.allow_always = True
    if self.CP.brand == "tesla":
      self.allow_always = True

    if self.CP.brand in MADS_NO_ACC_MAIN_BUTTON:
      self.no_main_cruise = True

    # Initialize parameters
    self.enabled_toggle: bool = self.params.get_bool("Mads")
    self.main_enabled_toggle: bool = self.params.get_bool("MadsMainCruiseAllowed")
    self.steering_mode_on_brake: MadsSteeringModeOnBrake = read_steering_mode_param(self.CP, self.CP_SP, self.params)
    self.unified_engagement_mode: bool = self.params.get_bool("MadsUnifiedEngagementMode")

  def read_params(self) -> None:
    """
    Read MADS-related parameters that may change during runtime.
    This method should be called periodically to update parameter values.
    """
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

  def pedal_pressed_non_gas_pressed(self, CS: structs.CarState) -> bool:
    """
    Check if brake pedal is pressed without gas pedal being pressed.
    This distinguishes between brake presses and gas pedal events that might trigger
    pedalPressed events.

    Args:
      CS: Current car state

    Returns:
      True if brake is pressed without gas being the cause of the pedalPressed event
    """
    # ignore `pedalPressed` events caused by gas presses
    if self.events.has(EventName.pedalPressed) and not (CS.gasPressed and not self.selfdrive.CS_prev.gasPressed and self.disengage_on_accelerator):
      return True

    return False

  def should_silent_lkas_enable(self, CS: structs.CarState) -> bool:
    """
    Determine if LKAS should be enabled silently based on brake state and gear position.

    Args:
      CS: Current car state

    Returns:
      True if LKAS should be enabled silently
    """
    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE and self.pedal_pressed_non_gas_pressed(CS):
      return False

    if self.events_sp.contains_in_list(GEARS_ALLOW_PAUSED_SILENT):
      return False

    return True

  def block_unified_engagement_mode(self) -> bool:
    """
    Check if unified engagement mode should block engagement.

    Returns:
      True if unified engagement should block engagement
    """
    # UEM disabled
    if not self.unified_engagement_mode:
      return True

    if self.enabled:
      return True

    if self.selfdrive.enabled and self.enabled_prev:
      return True

    return False

  def get_wrong_car_mode(self, alert_only: bool) -> None:
    """
    Handle wrong car mode events based on whether to show alert only or block control.

    Args:
      alert_only: If True, show alert only; if False, remove the event entirely
    """
    if alert_only:
      if self.events.has(EventName.wrongCarMode):
        self.replace_event(EventName.wrongCarMode, EventNameSP.wrongCarModeAlertOnly)
    else:
      self.events.remove(EventName.wrongCarMode)

  def transition_paused_state(self) -> None:
    """
    Transition to paused state by adding the appropriate silent event.
    This is called when the system needs to go to a paused state instead of fully disabling.
    """
    if self.state_machine.state != State.paused:
      self.events_sp.add(EventNameSP.silentLkasDisable)

  def replace_event(self, old_event: int, new_event: int) -> None:
    """
    Replace one event with another event in the event systems.

    Args:
      old_event: The event to be removed
      new_event: The event to be added instead
    """
    self.events.remove(old_event)
    self.events_sp.add(new_event)

  def update_events(self, CS: structs.CarState) -> None:
    """
    Update MADS-specific events based on car state and system conditions.

    This method handles various scenarios including:
    - Standstill conditions (door open, seatbelt unlatched)
    - Gear-related events (wrong gear, reverse gear)
    - Brake handling based on configured mode
    - Cruise control integration
    - Button event processing
    """
    if not self.selfdrive.enabled and self.enabled:
      if CS.standstill:
        if self.events.has(EventName.doorOpen):
          self.replace_event(EventName.doorOpen, EventNameSP.silentDoorOpen)
          self.transition_paused_state()
        if self.events.has(EventName.seatbeltNotLatched):
          self.replace_event(EventName.seatbeltNotLatched, EventNameSP.silentSeatbeltNotLatched)
          self.transition_paused_state()
      if self.events.has(EventName.wrongGear) and (CS.vEgo < 2.5 or CS.gearShifter == GearShifter.reverse):
        self.replace_event(EventName.wrongGear, EventNameSP.silentWrongGear)
        self.transition_paused_state()
      if self.events.has(EventName.reverseGear):
        self.replace_event(EventName.reverseGear, EventNameSP.silentReverseGear)
        self.transition_paused_state()
      if self.events.has(EventName.brakeHold):
        self.replace_event(EventName.brakeHold, EventNameSP.silentBrakeHold)
        self.transition_paused_state()
      if self.events.has(EventName.parkBrake):
        self.replace_event(EventName.parkBrake, EventNameSP.silentParkBrake)
        self.transition_paused_state()

      if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE:
        if self.pedal_pressed_non_gas_pressed(CS):
          self.transition_paused_state()
          self.events_sp.add(EventNameSP.madsPaused)

      # Remove standard events that are handled by MADS
      self.events.remove(EventName.preEnableStandstill)
      self.events.remove(EventName.belowEngageSpeed)
      self.events.remove(EventName.speedTooLow)
      self.events.remove(EventName.cruiseDisabled)
      self.events.remove(EventName.manualRestart)

    # Check if openpilot is trying to enable
    selfdrive_enable_events = self.events.has(EventName.pcmEnable) or self.events.has(EventName.buttonEnable)
    set_speed_btns_enable = any(be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents)

    # wrongCarMode alert only or actively block control
    self.get_wrong_car_mode(selfdrive_enable_events or set_speed_btns_enable)

    if selfdrive_enable_events:
      if self.pedal_pressed_non_gas_pressed(CS):
        self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

      if self.block_unified_engagement_mode():
        # Add clearer feedback when unified engagement mode blocks engagement
        if self.unified_engagement_mode and not self.enabled and self.selfdrive.enabled:
          self.events_sp.add(EventNameSP.manualLongitudinalRequired)
        self.events.remove(EventName.pcmEnable)
        self.events.remove(EventName.buttonEnable)
    else:
      if self.main_enabled_toggle:
        if CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
          self.events_sp.add(EventNameSP.lkasEnable)

    # Process button events for MADS control with clearer logic
    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel:
        if not self.selfdrive.enabled and self.enabled_prev:
          self.events_sp.add(EventNameSP.manualLongitudinalRequired)
      elif be.type == ButtonType.lkas and be.pressed and (CS.cruiseState.available or self.allow_always):
        # Provide clearer feedback based on current state
        if self.enabled:
          if self.selfdrive.enabled:
            # MADS is enabled but openpilot is also active - warn user
            self.events_sp.add(EventNameSP.manualSteeringRequired)
          else:
            # MADS is enabled but openpilot is not - proper disengage
            self.events_sp.add(EventNameSP.lkasDisable)
        else:
          # MADS is not enabled - enable it
          self.events_sp.add(EventNameSP.lkasEnable)

    # Handle cruise availability changes with clearer feedback
    if not CS.cruiseState.available and not self.no_main_cruise:
      self.events.remove(EventName.buttonEnable)
      if self.selfdrive.CS_prev.cruiseState.available:
        self.events_sp.add(EventNameSP.lkasDisable)

    # Handle brake-based steering behavior with clearer logic
    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.DISENGAGE:
      if self.pedal_pressed_non_gas_pressed(CS):
        if self.enabled:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          # block lkasEnable if being sent, then send pedalPressedAlertOnly event
          if self.events_sp.contains(EventNameSP.lkasEnable):
            self.events_sp.remove(EventNameSP.lkasEnable)
            self.events_sp.add(EventNameSP.pedalPressedAlertOnly)
    elif self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE:
      # Handle pause behavior on brake
      if self.pedal_pressed_non_gas_pressed(CS):
        if self.enabled and self.state_machine.state != State.paused:
          # Only pause if currently enabled and not already paused
          self.transition_paused_state()
          self.events_sp.add(EventNameSP.madsPaused)

    # Enable silent LKAS when appropriate
    if self.should_silent_lkas_enable(CS):
      if self.state_machine.state == State.paused:
        self.events_sp.add(EventNameSP.silentLkasEnable)

    # Clean up standard events that should be handled by MADS
    self.events.remove(EventName.pcmDisable)
    self.events.remove(EventName.buttonCancel)
    self.events.remove(EventName.pedalPressed)
    self.events.remove(EventName.wrongCruiseMode)

  def update(self, CS: structs.CarState) -> None:
    """
    Main update method for MADS system to process car state and update internal state.

    Args:
      CS: Current car state
    """
    if not self.enabled_toggle:
      # When MADS is not enabled, ensure we don't interfere with standard operation
      self.enabled = False
      self.active = False
      self.enabled_prev = self.selfdrive.enabled
      return

    self.update_events(CS)

    if not self.CP.passive and self.selfdrive.initialized:
      self.enabled, self.active = self.state_machine.update()
    else:
      # Ensure MADS is properly disabled when car is passive or selfdrive not initialized
      self.enabled = False
      self.active = False

    # Copy of previous SelfdriveD states for MADS events handling
    self.enabled_prev = self.selfdrive.enabled
