import pytest
from pytest_mock import MockerFixture

from cereal import custom, log # Import log for EventName
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.mads.state import StateMachine, SOFT_DISABLE_TIME
from openpilot.selfdrive.selfdrived.events import ET, Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

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
        event_name_to_add = EventName.startup # Use a dummy event name
        self.events_sp.add(event_name_to_add)
        self.events_sp._event_mappings[event_name_to_add][ET.IMMEDIATE_DISABLE] = None # Simulate mapping

        self.state_machine.state = state
        self.state_machine.update()
        assert State.disabled == self.state_machine.state

  def test_user_disable(self):
    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.clear_events()
        event_name_to_add = EventName.startup # Use a dummy event name
        self.events_sp.add(event_name_to_add)
        self.events_sp._event_mappings[event_name_to_add][ET.USER_DISABLE] = None # Simulate mapping

        self.state_machine.state = state
        self.state_machine.update()
        assert State.disabled == self.state_machine.state

  def test_user_disable_to_paused(self):
    paused_events = (EventNameSP.silentLkasDisable, )
    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.clear_events()
        event_name_to_add = EventName.startup # Use a dummy event name
        self.events_sp.add(event_name_to_add)
        self.events_sp._event_mappings[event_name_to_add][ET.USER_DISABLE] = None # Simulate mapping

        for en in paused_events:
          self.events_sp.add(en) # Add silentLkasDisable directly

          self.state_machine.state = state
          self.state_machine.update()
          final_state = State.paused if self.events_sp.has(en) and state != State.disabled else State.disabled
          assert self.state_machine.state == final_state

  def test_soft_disable(self):
    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.clear_events()
        event_name_to_add = EventName.startup # Use a dummy event name
        self.events_sp.add(event_name_to_add)
        self.events_sp._event_mappings[event_name_to_add][ET.SOFT_DISABLE] = None # Simulate mapping

        self.state_machine.state = state
        self.state_machine.update()
        assert self.state_machine.state == State.disabled if state == State.disabled else State.softDisabling

  def test_soft_disable_timer(self):
    self.state_machine.state = State.enabled
    self.clear_events()
    event_name_to_add = EventName.startup # Use a dummy event name
    self.events_sp.add(event_name_to_add)
    self.events_sp._event_mappings[event_name_to_add][ET.SOFT_DISABLE] = None # Simulate mapping

    self.state_machine.update()
    for _ in range(int(SOFT_DISABLE_TIME / DT_CTRL)):
      assert self.state_machine.state == State.softDisabling
      self.mads.selfdrive.state_machine.soft_disable_timer -= 1
      self.state_machine.update()

    assert self.state_machine.state == State.disabled

  def test_no_entry(self):
    for et_event_type in ENABLE_EVENT_TYPES:
      self.clear_events()
      event_name_to_add = EventName.startup # Use a dummy event name
      self.events_sp.add(event_name_to_add)
      self.events_sp._event_mappings[event_name_to_add][ET.NO_ENTRY] = None
      self.events_sp._event_mappings[event_name_to_add][et_event_type] = None

      self.state_machine.update()
      assert self.state_machine.state == State.disabled

  def test_no_entry_paused(self):
    self.state_machine.state = State.paused
    self.clear_events()
    event_name_to_add = EventName.startup # Use a dummy event name
    self.events_sp.add(event_name_to_add)
    self.events_sp._event_mappings[event_name_to_add][ET.NO_ENTRY] = None

    self.state_machine.update()
    assert self.state_machine.state == State.paused

  def test_override_lateral(self):
    self.state_machine.state = State.enabled
    self.clear_events()
    event_name_to_add = EventName.startup # Use a dummy event name
    self.events_sp.add(event_name_to_add)
    self.events_sp._event_mappings[event_name_to_add][ET.OVERRIDE_LATERAL] = None

    self.state_machine.update()
    assert self.state_machine.state == State.overriding

  def test_paused_to_enabled(self):
    self.state_machine.state = State.paused
    self.clear_events()
    event_name_to_add = EventName.startup # Use a dummy event name
    self.events_sp.add(event_name_to_add)
    self.events_sp._event_mappings[event_name_to_add][ET.ENABLE] = None

    self.state_machine.update()
    assert self.state_machine.state == State.enabled

  def test_maintain_states(self):
    for state in ALL_STATES:
      for et_event_type in MAINTAIN_STATES[state]:
        self.state_machine.state = state
        self.clear_events()
        if et_event_type is not None:
          event_name_to_add = EventName.startup # Use a dummy event name
          self.events_sp.add(event_name_to_add)
          self.events_sp._event_mappings[event_name_to_add][et_event_type] = None

        self.state_machine.update()
        assert self.state_machine.state == state
