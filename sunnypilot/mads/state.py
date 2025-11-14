"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import logging
from typing import Dict, Any

from cereal import log, custom
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.selfdrived.state import SOFT_DISABLE_TIME
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName

ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.paused, *ACTIVE_STATES)

GEARS_ALLOW_PAUSED_SILENT = [EventNameSP.silentWrongGear, EventNameSP.silentReverseGear, EventNameSP.silentBrakeHold,
                             EventNameSP.silentDoorOpen, EventNameSP.silentSeatbeltNotLatched, EventNameSP.silentParkBrake]
GEARS_ALLOW_PAUSED = [EventName.wrongGear, EventName.reverseGear, EventName.brakeHold,
                      EventName.doorOpen, EventName.seatbeltNotLatched, EventName.parkBrake]


class StateMachine:
  """
  MADS (Modular Assisted Driving System) state machine.

  This state machine manages the state transitions for the sunnypilot's additional driving features
  that extend beyond the standard openpilot behavior. It works in conjunction with the main
  openpilot state machine to provide enhanced functionality while maintaining safety.

  State transitions:
  - disabled: Default state when MADS is not active
  - enabled: MADS is active and controlling the vehicle
  - softDisabling: MADS is in the process of soft disengagement
  - overriding: User is overriding MADS control
  - paused: MADS is temporarily paused due to vehicle conditions

  The state machine integrates with openpilot's event system and alert management to ensure
  consistent behavior with the base system.
  """
  def __init__(self, mads):
    self.selfdrive = mads.selfdrive
    self.ss_state_machine = mads.selfdrive.state_machine
    self._events = mads.selfdrive.events
    self._events_sp = mads.selfdrive.events_sp

    self.state = State.disabled
    self._prev_state = State.disabled  # Track previous state for better logging
    self._state_transition_count = 0  # Track transition frequency for debugging

  def add_current_alert_types(self, alert_type) -> None:
    """
    Add alert type to the main selfdrive state machine's current alert types.

    Args:
        alert_type: The alert type to add
    """
    try:
      if not self.selfdrive.enabled:
        self.ss_state_machine.current_alert_types.append(alert_type)
    except AttributeError as e:
      cloudlog.error(f"MADS StateMachine: AttributeError in add_current_alert_types: {e}")
      # Fallback to safe behavior
      if hasattr(self, 'ss_state_machine') and hasattr(self.ss_state_machine, 'current_alert_types'):
        self.ss_state_machine.current_alert_types.append(alert_type)

  def check_contains(self, event_type: str) -> bool:
    """
    Check if an event type exists in either the main events or SP events.

    Args:
        event_type: The event type to check for

    Returns:
        True if the event type exists in either event collection
    """
    try:
      return bool(self._events.contains(event_type) or self._events_sp.contains(event_type))
    except AttributeError as e:
      cloudlog.error(f"MADS StateMachine: AttributeError in check_contains: {e}")
      # Return False as safe fallback
      return False

  def check_contains_in_list(self) -> bool:
    """
    Check if any events from the allowed paused lists are present.

    Returns:
        True if any gear-related pause events are present
    """
    try:
      return bool(self._events.contains_in_list(GEARS_ALLOW_PAUSED) or self._events_sp.contains_in_list(GEARS_ALLOW_PAUSED_SILENT))
    except AttributeError as e:
      cloudlog.error(f"MADS StateMachine: AttributeError in check_contains_in_list: {e}")
      # Return False as safe fallback
      return False

  def _get_detailed_event_info(self) -> Dict[str, Any]:
    """
    Get detailed information about current events for debugging.

    Returns:
        Dictionary with event details for debugging purposes
    """
    try:
      event_info = {
        "user_disable": self._events.contains(ET.USER_DISABLE) or self._events_sp.contains(ET.USER_DISABLE),
        "immediate_disable": self._events.contains(ET.IMMEDIATE_DISABLE) or self._events_sp.contains(ET.IMMEDIATE_DISABLE),
        "enable": self._events.contains(ET.ENABLE) or self._events_sp.contains(ET.ENABLE),
        "no_entry": self._events.contains(ET.NO_ENTRY) or self._events_sp.contains(ET.NO_ENTRY),
        "soft_disable": self._events.contains(ET.SOFT_DISABLE) or self._events_sp.contains(ET.SOFT_DISABLE),
        "override_lateral": self._events.contains(ET.OVERRIDE_LATERAL) or self._events_sp.contains(ET.OVERRIDE_LATERAL),
        "in_gear_allow_paused": self.check_contains_in_list(),
        "sp_silent_lkas_disable": self._events_sp.has(EventNameSP.silentLkasDisable)
      }
      return event_info
    except Exception as e:
      cloudlog.error(f"MADS StateMachine: Error getting event info: {e}")
      return {}

  def update(self) -> tuple[bool, bool]:
    """
    Update the MADS state based on current events and conditions.

    This method implements the state transition logic for MADS, integrating with
    openpilot's main state machine and event system. It handles priority-based
    state transitions and manages the soft-disable timer.

    Returns:
        A tuple of (enabled, active) status:
        - enabled: True if MADS is enabled (in any state other than disabled)
        - active: True if MADS is actively controlling the vehicle
    """
    try:
      # Store the previous state for comparison
      self._prev_state = self.state

      # Get detailed event information for logging
      event_info = self._get_detailed_event_info()

      # soft disable timer and current alert types are from the state machine of openpilot
      # decrement the soft disable timer at every step, as it's reset on
      # entrance in SOFT_DISABLING state

      # Priority 1: Disable events always take precedence
      if self.check_contains(ET.USER_DISABLE):
        if self._events_sp.has(EventNameSP.silentLkasDisable):
          self.state = State.paused
          cloudlog.debug("MADS: Transition to paused due to silent LKAS disable")
        else:
          self.state = State.disabled
          cloudlog.debug("MADS: Transition to disabled due to user disable")
        self.ss_state_machine.current_alert_types.append(ET.USER_DISABLE)
      elif self.check_contains(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        cloudlog.debug("MADS: Transition to disabled due to immediate disable")
        self.add_current_alert_types(ET.IMMEDIATE_DISABLE)
      else:
        # Process state transitions based on current state
        if self.state == State.disabled:
          # Check if we should transition to an enabled state
          if self.check_contains(ET.ENABLE):
            if self.check_contains(ET.NO_ENTRY):
              if self.check_contains_in_list():
                self.state = State.paused
                cloudlog.debug("MADS: Transition to paused due to gear conditions")
              self.add_current_alert_types(ET.NO_ENTRY)
            else:
              if self.check_contains(ET.OVERRIDE_LATERAL):
                self.state = State.overriding
                cloudlog.debug("MADS: Transition to overriding due to lateral override")
              else:
                self.state = State.enabled
                cloudlog.debug("MADS: Transition to enabled")
              self.add_current_alert_types(ET.ENABLE)

        elif self.state in (State.enabled, State.softDisabling, State.overriding, State.paused):
          # Process transitions from non-disabled states
          if self.state == State.enabled:
            if self.check_contains(ET.SOFT_DISABLE):
              self.state = State.softDisabling
              cloudlog.debug("MADS: Transition to softDisabling due to soft disable")
              if not self.selfdrive.enabled:
                self.ss_state_machine.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
                self.ss_state_machine.current_alert_types.append(ET.SOFT_DISABLE)
            elif self.check_contains(ET.OVERRIDE_LATERAL):
              self.state = State.overriding
              cloudlog.debug("MADS: Transition to overriding due to lateral override")
              self.add_current_alert_types(ET.OVERRIDE_LATERAL)

          elif self.state == State.softDisabling:
            if not self.check_contains(ET.SOFT_DISABLE):
              # no more soft disabling condition, so go back to ENABLED
              self.state = State.enabled
              cloudlog.debug("MADS: Transition to enabled from softDisabling (condition cleared)")
            elif self.ss_state_machine.soft_disable_timer > 0:
              self.add_current_alert_types(ET.SOFT_DISABLE)
            elif self.ss_state_machine.soft_disable_timer <= 0:
              self.state = State.disabled
              cloudlog.debug("MADS: Transition to disabled from softDisabling (timer expired)")

          elif self.state == State.paused:
            if self.check_contains(ET.ENABLE):
              if self.check_contains(ET.NO_ENTRY):
                self.add_current_alert_types(ET.NO_ENTRY)
                cloudlog.debug("MADS: Staying paused due to NO_ENTRY")
              else:
                if self.check_contains(ET.OVERRIDE_LATERAL):
                  self.state = State.overriding
                  cloudlog.debug("MADS: Transition to overriding from paused")
                else:
                  self.state = State.enabled
                  cloudlog.debug("MADS: Transition to enabled from paused")
                self.add_current_alert_types(ET.ENABLE)

          elif self.state == State.overriding:
            if self.check_contains(ET.SOFT_DISABLE):
              self.state = State.softDisabling
              cloudlog.debug("MADS: Transition to softDisabling from overriding")
              if not self.selfdrive.enabled:
                self.ss_state_machine.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
                self.ss_state_machine.current_alert_types.append(ET.SOFT_DISABLE)
            elif not self.check_contains(ET.OVERRIDE_LATERAL):
              self.state = State.enabled
              cloudlog.debug("MADS: Transition to enabled from overriding")
            else:
              self.ss_state_machine.current_alert_types += [ET.OVERRIDE_LATERAL]

      # Check if MADS is engaged and actuators are enabled
      enabled = self.state in ENABLED_STATES
      active = self.state in ACTIVE_STATES
      if active:
        self.add_current_alert_types(ET.WARNING)

      # Log state changes for debugging and user experience insight
      if self.state != self._prev_state:
        self._state_transition_count += 1
        cloudlog.debug(f"MADS state changed from {self._prev_state} to {self.state}, "
                      f"transition #{self._state_transition_count}, events: {event_info}")

      return enabled, active

    except Exception as e:
      cloudlog.error(f"MADS StateMachine update error: {e}")
      # Return safe defaults in case of error
      enabled = self.state in ENABLED_STATES
      active = self.state in ACTIVE_STATES
      return enabled, active

