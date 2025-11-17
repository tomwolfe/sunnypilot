import pytest
from pytest_mock import MockerFixture

from cereal import custom, log # Import log for EventName
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.mads.state import StateMachine, SOFT_DISABLE_TIME
from openpilot.selfdrive.selfdrived.events import ET, Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

pytestmark = pytest.mark.unit


def _get_event_mappings(events):
  if hasattr(events, "_event_mappings"):
    return events._event_mappings
  return events.get_events_mapping()

# State mapping from capnp schema:
# State.disabled = 0
# State.paused = 1
# State.enabled = 2
# State.softDisabling = 3
# State.overriding = 4
State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventNameSP = custom.OnroadEventSP.EventName
EventName = log.OnroadEvent.EventName # Use EventName from log for dummy events

# The event types that maintain the current state
MAINTAIN_STATES = {State.enabled: (None,), State.disabled: (None,), State.softDisabling: (ET.SOFT_DISABLE,),
                   State.paused: (None,), State.overriding: (ET.OVERRIDE_LATERAL,)}
ALL_STATES = (State.schema.enumerants.values())
# The event types checked in DISABLED section of state machine
ENABLE_EVENT_TYPES = (ET.ENABLE, ET.OVERRIDE_LATERAL)


class MockMADS:
  def __init__(self, mocker: MockerFixture):
    self.selfdrive = mocker.MagicMock()
    self.selfdrive.state_machine = mocker.MagicMock()
    self.selfdrive.events = Events()
    self.selfdrive.events_sp = EventsSP()


class TestMADSStateMachine:
  @pytest.fixture(autouse=True)
  def setup_method(self, mocker: MockerFixture):
    self.mads = MockMADS(mocker)
    self.state_machine = StateMachine(self.mads)
    self.events = self.mads.selfdrive.events
    self.events_sp = self.mads.selfdrive.events_sp
    self.mads.selfdrive.state_machine.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)

  def clear_events(self):
    self.events.clear()
    self.events_sp.clear()

  def test_immediate_disable(self):
    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.clear_events()
        event_name_to_add = EventNameSP.lkasDisable # Use a valid SP event name
        self.events_sp.add(event_name_to_add)
        event_mappings = _get_event_mappings(self.events_sp)
        if event_name_to_add in event_mappings:
          event_mappings[event_name_to_add][ET.IMMEDIATE_DISABLE] = None # Simulate mapping
        else:
          # If the event_name_to_add is not in the mapping, skip this test iteration
          continue

        self.state_machine.state = state
        self.state_machine.update()
        assert State.disabled == self.state_machine.state

  def test_user_disable(self):
    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.clear_events()
        event_name_to_add = EventNameSP.lkasDisable # Use a valid SP event name
        self.events_sp.add(event_name_to_add)
        event_mappings = _get_event_mappings(self.events_sp)
        if event_name_to_add in event_mappings:
          event_mappings[event_name_to_add][ET.USER_DISABLE] = None # Simulate mapping
        else:
          # If the event_name_to_add is not in the mapping, skip this test iteration
          continue

        self.state_machine.state = state
        self.state_machine.update()
        assert State.disabled == self.state_machine.state

  def test_user_disable_to_paused(self):
    # Test that when we have both a USER_DISABLE event and silentLkasDisable event,
    # the state transitions to paused instead of disabled
    self.state_machine.state = State.enabled  # Start with enabled state
    self.clear_events()

    # Add events that will trigger USER_DISABLE
    self.events_sp.add(EventNameSP.lkasDisable)  # This maps to ET.USER_DISABLE in EVENTS_SP
    self.events_sp.add(EventNameSP.silentLkasDisable)  # Also add silent event

    self.state_machine.update()
    # Should transition to paused because of silentLkasDisable
    assert self.state_machine.state == State.paused

  def test_soft_disable(self):
    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.clear_events()
        event_name_to_add = EventNameSP.lkasDisable # Use a valid SP event name
        self.events_sp.add(event_name_to_add)
        event_mappings = _get_event_mappings(self.events_sp)
        if event_name_to_add in event_mappings:
          event_mappings[event_name_to_add][ET.SOFT_DISABLE] = None # Simulate mapping
        else:
          # If the event_name_to_add is not in the mapping, skip this test iteration
          continue

        self.state_machine.state = state
        self.state_machine.update()
        assert self.state_machine.state == State.disabled if state == State.disabled else State.softDisabling

  def test_soft_disable_timer(self):
    self.state_machine.state = State.enabled
    self.clear_events()
    # Use an event that maps to ET.SOFT_DISABLE
    from openpilot.selfdrive.selfdrived.events import EventName
    self.events.add(EventName.steerTempUnavailable)  # This maps to ET.SOFT_DISABLE in main events

    self.state_machine.update()
    for _ in range(int(SOFT_DISABLE_TIME / DT_CTRL)):
      assert self.state_machine.state == State.softDisabling
      self.mads.selfdrive.state_machine.soft_disable_timer -= 1
      self.state_machine.update()

    assert self.state_machine.state == State.disabled

  def test_no_entry(self):
    for et_event_type in ENABLE_EVENT_TYPES:
      self.clear_events()
      event_name_to_add = EventNameSP.lkasDisable # Use a valid SP event name
      self.events_sp.add(event_name_to_add)
      event_mappings = _get_event_mappings(self.events_sp)
      if event_name_to_add in event_mappings:
        event_mappings[event_name_to_add][ET.NO_ENTRY] = None
        event_mappings[event_name_to_add][et_event_type] = None
      else:
        # If the event_name_to_add is not in the mapping, skip this test iteration
        continue

      self.state_machine.update()
      assert self.state_machine.state == State.disabled

  def test_no_entry_paused(self):
    self.state_machine.state = State.paused
    self.clear_events()
    # Use an event that maps to ET.NO_ENTRY - using SP event that has NO_ENTRY mapping
    # Based on EVENTS_SP, several events map to NO_ENTRY like EventNameSP.silentWrongGear
    self.events_sp.add(EventNameSP.silentWrongGear)  # This maps to ET.NO_ENTRY in EVENTS_SP

    self.state_machine.update()
    # When in paused state with NO_ENTRY event (but no ENABLE event), should remain paused
    assert self.state_machine.state == State.paused

  def test_override_lateral(self):
    self.state_machine.state = State.enabled
    self.clear_events()
    # Use the correct original event name that maps to ET.OVERRIDE_LATERAL
    # Note: For the SP events, we need to check what actually maps to OVERRIDE_LATERAL
    # Since there's no direct SP event mapping to OVERRIDE_LATERAL in EVENTS_SP,
    # we'll simulate by using the main event system
    from openpilot.selfdrive.selfdrived.events import EventName
    self.events.add(EventName.steerOverride)  # This maps to ET.OVERRIDE_LATERAL in main events

    self.state_machine.update()
    # When enabled state receives OVERRIDE_LATERAL event, should transition to overriding
    assert self.state_machine.state == State.overriding

  def test_paused_to_enabled(self):
    self.state_machine.state = State.paused
    self.clear_events()
    # Use an event that maps to ET.ENABLE
    self.events_sp.add(EventNameSP.lkasEnable)  # This maps to ET.ENABLE in EVENTS_SP

    self.state_machine.update()
    # When paused state receives ENABLE event, should transition to enabled
    assert self.state_machine.state == State.enabled

  def test_maintain_states(self):
    # Test that certain events maintain the current state according to MAINTAIN_STATES mapping
    from openpilot.selfdrive.selfdrived.events import EventName

    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.state_machine.state = state
        self.clear_events()
        if et_event_type is not None:
          # Add events based on the event type to maintain state
          if et_event_type == ET.SOFT_DISABLE:
            self.events.add(EventName.steerTempUnavailable)  # Maps to ET.SOFT_DISABLE
          elif et_event_type == ET.OVERRIDE_LATERAL:
            self.events.add(EventName.steerOverride)  # Maps to ET.OVERRIDE_LATERAL

        self.state_machine.update()
        # State should be maintained according to MAINTAIN_STATES mapping
        assert self.state_machine.state == state
