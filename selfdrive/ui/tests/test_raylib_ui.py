import time
import signal
import pytest
from openpilot.selfdrive.test.helpers import with_processes, processes_context
from openpilot.system.manager.process_config import managed_processes


def test_raylib_ui():
  """Test initialization of the UI widgets is successful."""
  with processes_context(["ui"], init_time=0) as procs:
    try:
      time.sleep(1)
    finally:
      # Ensure UI process is properly terminated
      ui_proc = managed_processes["ui"]
      if ui_proc.proc is not None and ui_proc.proc.pid is not None:
        # Send SIGTERM to allow atexit handlers to run
        try:
          ui_proc.stop(sig=signal.SIGTERM, block=True)
        except Exception:
          pass
