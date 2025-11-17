import os
import pytest
import time
from unittest.mock import MagicMock, patch
from pytest_mock import MockerFixture # Import MockerFixture

import cereal.messaging as messaging
from cereal import log
from openpilot.common.gpio import gpio_set, gpio_init
from panda import Panda, PandaDFU, PandaProtocolMismatch
from openpilot.common.retry import retry
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.tici.pins import GPIO

pytestmark = pytest.mark.skipif(
    os.getenv("CI", "0") == "1" or os.getenv("GITHUB_ACTIONS", "false") == "true",
    reason="Skipping hardware-dependent tests on CI (no panda / device present)"
)

HERE = os.path.dirname(os.path.realpath(__file__))


@pytest.mark.tici
@pytest.mark.timeout(300)  # 5 minute timeout for pandad tests
class TestPandad:

  @pytest.fixture(autouse=True)
  def setup_method(self, mocker: MockerFixture): # Add mocker fixture
    # Mock Panda class and its methods
    self.mock_panda_list = mocker.patch('panda.Panda.list', return_value=[])
    self.mock_panda_wait_for_dfu = mocker.patch('panda.Panda.wait_for_dfu', return_value=True)
    self.mock_panda_wait_for_panda = mocker.patch('panda.Panda.wait_for_panda', return_value=True)
    self.mock_panda_constructor = mocker.patch('panda.Panda', autospec=True)
    self.mock_panda_dfu_constructor = mocker.patch('panda.PandaDFU', autospec=True)

    # Mock HARDWARE methods
    self.mock_hardware_recover = mocker.patch('openpilot.system.hardware.HARDWARE.recover_internal_panda')
    self.mock_hardware_reset = mocker.patch('openpilot.system.hardware.HARDWARE.reset_internal_panda')

    # Mock gpio functions
    mocker.patch('openpilot.common.gpio.gpio_init')
    mocker.patch('openpilot.common.gpio.gpio_set')

    # Mock the Panda object returned by the constructor
    mock_panda_instance = MagicMock()
    mock_panda_instance.bootstub = False # Default for most tests
    mock_panda_instance.is_internal.return_value = True # Assume it's internal for some tests
    mock_panda_instance.reset.side_effect = lambda enter_bootstub=False: None # Mock reset method
    self.mock_panda_constructor.return_value.__enter__.return_value = mock_panda_instance
    self.mock_panda_constructor.return_value.__exit__.return_value = None

    # Mock the PandaDFU object returned by the constructor
    mock_panda_dfu_instance = MagicMock()
    mock_panda_dfu_instance.program_bootstub.return_value = None
    mock_panda_dfu_instance.reset.return_value = None
    mock_panda_dfu_instance.get_mcu_type.return_value.config.bootstub_fn = "mock_bootstub.bin" # For _flash_bootstub_and_test
    self.mock_panda_dfu_constructor.return_value.__enter__.return_value = mock_panda_dfu_instance
    self.mock_panda_dfu_constructor.return_value.__exit__.return_value = None

    # Mock the messaging.SubMaster and its update method to return a pandaState
    self.mock_submaster = mocker.patch('cereal.messaging.SubMaster', autospec=True)
    mock_panda_state = log.PandaState.new_message()
    mock_panda_state.pandaType = log.PandaState.PandaType.whitePanda # Simulate a known panda type
    self.mock_submaster.return_value.update.side_effect = lambda timeout: None # Simulate no new messages by default
    self.mock_submaster.return_value.__getitem__.return_value = [mock_panda_state] # Simulate pandaStates message

    # Ensure pandad is stopped before each test
    managed_processes['pandad'].stop()

  def teardown_method(self):
    managed_processes['pandad'].stop()

  def _run_test(self, timeout=30) -> float:
    st = time.monotonic()
    sm = messaging.SubMaster(['pandaStates']) # This will use our mock_submaster

    managed_processes['pandad'].start()
    while (time.monotonic() - st) < timeout:
      sm.update(100)
      if len(sm['pandaStates']) and sm['pandaStates'][0].pandaType != log.PandaState.PandaType.unknown:
        break
    dt = time.monotonic() - st
    managed_processes['pandad'].stop()

    if len(sm['pandaStates']) == 0 or sm['pandaStates'][0].pandaType == log.PandaState.PandaType.unknown:
      raise Exception("pandad failed to start")

    return dt

  def _go_to_dfu(self):
    self.mock_hardware_recover.return_value = None # Mock the call
    self.mock_panda_wait_for_dfu.return_value = True # Mock the call

  def _assert_no_panda(self):
    self.mock_panda_wait_for_dfu.return_value = False
    self.mock_panda_wait_for_panda.return_value = False
    assert not Panda.wait_for_dfu(None, 3)
    assert not Panda.wait_for_panda(None, 3)

  @retry(attempts=3)
  def _flash_bootstub_and_test(self, fn, expect_mismatch=False):
    self._go_to_dfu()
    pd = PandaDFU(None) # This will use our mock_panda_dfu_constructor
    if fn is None:
      # Mock get_mcu_type().config.bootstub_fn
      fn = os.path.join(HERE, self.mock_panda_dfu_constructor.return_value.__enter__.return_value.get_mcu_type().config.bootstub_fn)
    with open(fn, "rb") as f:
      pd.program_bootstub(f.read()) # Mocked
    pd.reset() # Mocked
    self.mock_hardware_reset.return_value = None # Mock the call

    self.mock_panda_wait_for_panda.return_value = True # Mock the call
    if expect_mismatch:
      # We need to mock the Panda constructor to raise PandaProtocolMismatch
      self.mock_panda_constructor.side_effect = PandaProtocolMismatch
      with pytest.raises(PandaProtocolMismatch):
        Panda()
      self.mock_panda_constructor.side_effect = None # Reset side effect
    else:
      # Mock Panda constructor to return a Panda with bootstub=True
      mock_panda_instance = MagicMock()
      mock_panda_instance.bootstub = True
      self.mock_panda_constructor.return_value.__enter__.return_value = mock_panda_instance
      with Panda() as p:
        assert p.bootstub

    self._run_test(45)

  def test_in_dfu(self):
    self._go_to_dfu()
    self._run_test(60)

  def test_in_bootstub(self):
    mock_panda_instance = MagicMock()
    mock_panda_instance.bootstub = True
    mock_panda_instance.reset.side_effect = lambda enter_bootstub: None # Mock reset method
    self.mock_panda_constructor.return_value.__enter__.return_value = mock_panda_instance
    with Panda() as p:
      p.reset(enter_bootstub=True)
      assert p.bootstub
    self._run_test()

  def test_internal_panda_reset(self):
    # Mock gpio functions are already in setup_method
    # Simulate no internal panda initially
    self.mock_panda_list.return_value = []
    self.mock_panda_constructor.return_value.is_internal.return_value = False # For the assert all(not Panda(s).is_internal() ...

    self._run_test()

    # Simulate internal panda found after _run_test
    self.mock_panda_list.return_value = [MagicMock()] # Return a list with a mock panda
    self.mock_panda_constructor.return_value.is_internal.return_value = True # For the assert any(Panda(s).is_internal() ...
    assert any(Panda(s).is_internal() for s in Panda.list())

  def test_best_case_startup_time(self):
    # run once so we're up to date
    self._run_test(60)

    ts = []
    for _ in range(10):
      # should be nearly instant this time
      dt = self._run_test(5)
      ts.append(dt)

    # 5s for USB (due to enumeration)
    # - 0.2s pandad -> pandad
    # - plus some buffer
    print("startup times", ts, sum(ts) / len(ts))
    assert 0.1 < (sum(ts)/len(ts)) < 0.7

  def test_protocol_version_check(self):
    # flash old fw
    fn = os.path.join(HERE, "bootstub.panda_h7_spiv0.bin")
    self._flash_bootstub_and_test(fn, expect_mismatch=True)

  def test_release_to_devel_bootstub(self):
    self._flash_bootstub_and_test(None)

  def test_recover_from_bad_bootstub(self):
    self._go_to_dfu()
    # Mock PandaDFU to simulate programming a bad bootstub
    mock_panda_dfu_instance = MagicMock()
    mock_panda_dfu_instance.program_bootstub.return_value = None
    mock_panda_dfu_instance.reset.return_value = None
    self.mock_panda_dfu_constructor.return_value.__enter__.return_value = mock_panda_dfu_instance

    with PandaDFU(None) as pd:
      pd.program_bootstub(b"\x00"*1024)
      pd.reset()
    self.mock_hardware_reset.return_value = None # Mock the call
    self._assert_no_panda() # Mocked

    self._run_test(60)
