import os
import time
import pytest
from openpilot.system.hardware import PC
from openpilot.selfdrive.test.helpers import with_processes


@pytest.mark.skipif(os.environ.get("CI", False), reason="raylib_ui test hangs in CI environment")
@with_processes(["raylib_ui"])
def test_raylib_ui():
  """Test initialization of the UI widgets is successful."""
  time.sleep(1)
