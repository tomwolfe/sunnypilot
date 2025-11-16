#!/usr/bin/env python3
import os
import pyray as rl
from openpilot.common.watchdog import kick_watchdog
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.main import MainLayout
from openpilot.selfdrive.ui.ui_state import ui_state


def main():
  # Check if running in CI environment
  if os.environ.get("CI"):
    # In CI, run in headless mode without actual window
    import time
    from openpilot.selfdrive.ui.ui_state import ui_state

    # Initialize UI state without GUI
    ui_state.sm.init_threading()

    # Run a simple loop to keep the process alive and update UI state
    try:
      while True:
        ui_state.update()
        # Simulate the update frequency of the UI
        time.sleep(1.0 / 20.0)  # 20 Hz update rate to match UI_FREQ
        kick_watchdog()
    except KeyboardInterrupt:
      pass
  else:
    # Normal GUI mode for hardware
    gui_app.init_window("UI")
    main_layout = MainLayout()
    main_layout.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    for _ in gui_app.render():
      ui_state.update()

      # TODO handle brigntness and awake state here

      main_layout.render()

      kick_watchdog()


if __name__ == "__main__":
  main()
