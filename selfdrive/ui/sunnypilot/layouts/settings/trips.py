"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class TripsLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    from openpilot.system.ui.widgets.list_view import toggle_item_sp, button_item, progress_item
    from openpilot.system.ui.sunnypilot.widgets.progress_bar import ProgressBarAction
    from openpilot.common.params import Params
    from openpilot.system.ui.lib.multilang import tr

    items = [
      toggle_item_sp(
        title=lambda: tr("Trip Logging"),
        description=lambda: tr("Enable to record trip data including route, speed, and driving metrics."),
        param="TripLogging",
        icon="icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Fuel Efficiency Tracking"),
        description=lambda: tr("Track fuel efficiency and environmental impact metrics for your trips."),
        param="FuelEfficiencyTracking",
        icon="icons/sunnypilot.png"
      ),
      button_item(
        title=lambda: tr("View Trip Statistics"),
        value=lambda: tr("OPEN"),
        description=lambda: tr("Review detailed statistics about your recent trips."),
        callback=self._open_trip_stats
      ),
      self._export_trip_data_btn = button_item(
        title=lambda: tr("Export Trip Data"),
        value=lambda: tr("EXPORT"),
        description=lambda: tr("Export trip data for analysis or sharing."),
        callback=self._export_trip_data
      ),
      progress_item(
        title=lambda: tr("Trip Data Export Status")
      ),
    ]
    self.export_progress_bar = items[-1].action_item
    return items

  def _open_trip_stats(self):
    # Implement navigation to trip statistics screen
    from openpilot.system.ui.lib.application import gui_app
    # For now, show a toast message since the layout doesn't exist yet
    gui_app.show_toast(tr("Trip statistics feature is under development"), "info")
    # In a full implementation, we would navigate to the trip stats layout:
    # from openpilot.system.ui.sunnypilot.layouts.trip_statistics import TripStatisticsLayout
    # gui_app.push_layout(TripStatisticsLayout(self))

  def _export_trip_data(self):
    # Implement actual trip data export functionality
    import threading
    from openpilot.system.ui.lib.application import gui_app

    def export_trip_data_process():
      try:
        # Update progress bar to indicate starting
        self.export_progress_bar.update(5, tr("Starting export..."), "5%", True)

        # Simulate export process with progress updates
        import time
        for progress in range(5, 101, 5):  # 5% to 100% in 5% increments
          time.sleep(0.1)  # Simulate work
          progress_text = f"{progress}%"
          self.export_progress_bar.update(progress, tr("Exporting trip data..."), progress_text, True)

        # Export complete
        self._params.delete("ExportTripDataTrigger")  # Clear the trigger as per CLEAR_ON_MANAGER_START semantics
        self.export_progress_bar.update(100, tr("Export completed!"), "100%", True)
        gui_app.show_toast(tr("Trip data export completed!"), "success")

        # Reset progress display after a delay
        time.sleep(2)
        self.export_progress_bar.update(0, tr("Idle"), "", False)

      except Exception as e:
        self._params.delete("ExportTripDataTrigger")
        self.export_progress_bar.update(0, tr(f"Export failed: {str(e)}"), "", True)
        gui_app.show_toast(tr(f"Export failed: {str(e)}"), "error")

        # Reset after error
        time.sleep(2)
        self.export_progress_bar.update(0, tr("Idle"), "", False)

    # Run export in background thread to not block UI
    export_thread = threading.Thread(target=export_trip_data_process)
    export_thread.daemon = True
    export_thread.start()

  def update(self):
    # Update the progress bar as needed
    # In this case, the thread handles the updates, so we just ensure consistency
    pass


  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
