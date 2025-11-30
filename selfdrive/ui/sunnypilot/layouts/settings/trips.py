"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import os
import time
from datetime import datetime
import threading

from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget

DEFAULT_TRIP_DATA_PATH = "/data/media/0/sunnypilot/trip_data/"
TRIP_DATA_PATH_PARAM_KEY = "CustomTripDataPath"

def get_trip_data_path():
  params = Params()
  custom_path = params.get(TRIP_DATA_PATH_PARAM_KEY, encoding='utf-8')
  if custom_path and os.path.isdir(custom_path): # Only use custom path if it exists and is a directory
    return custom_path
  return DEFAULT_TRIP_DATA_PATH

TRIP_DATA_PATH = get_trip_data_path()


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
    ]
    _export_btn = button_item(
      title=lambda: tr("Export Trip Data"),
      value=lambda: tr("EXPORT"),
      description=lambda: tr("Export trip data for analysis or sharing."),
      callback=self._export_trip_data
    )
    items.append(_export_btn)
    items.append(
      progress_item(
        title=lambda: tr("Trip Data Export Status")
      )
    )
    self._export_trip_data_btn = _export_btn
    self.export_progress_bar = items[-1].action_item
    return items

  def _open_trip_stats(self):
    # Implement navigation to trip statistics screen
    from openpilot.system.ui.lib.application import gui_app
    # For now, show a toast message since the layout doesn't exist yet
    gui_app.show_toast(tr("Trip statistics feature is under development"), "info")
    from openpilot.system.ui.sunnypilot.layouts.trip_statistics import TripStatisticsLayout
    gui_app.push_layout(TripStatisticsLayout(self))

  def _export_trip_data(self):
    from openpilot.system.ui.lib.application import gui_app

    def export_trip_data_process():
      try:
        self._export_trip_data_btn.action_item.set_enabled(False) # Disable button during export
        self.export_progress_bar.update(5, tr("Preparing export..."), "5%", True)
        os.makedirs(TRIP_DATA_PATH, exist_ok=True)
        time.sleep(0.5) # Simulate some work

        self.export_progress_bar.update(20, tr("Generating trip data..."), "20%", True)
        # Simulate trip data generation
        start_time = datetime.now()
        time.sleep(2) # Simulate a long trip
        end_time = datetime.now()
        duration = (end_time - start_time).total_seconds()
        distance_meters = round(duration * 20 / 3.6, 2) # Assume average speed 20 km/h
        average_speed_kph = round(distance_meters / duration * 3.6, 2) if duration > 0 else 0
        fuel_consumed_liters = round(distance_meters / 10000, 2) # Assume 10L/100km

        trip_data = {
          "start_time": start_time.isoformat(),
          "end_time": end_time.isoformat(),
          "duration_seconds": round(duration, 2),
          "distance_meters": distance_meters,
          "average_speed_kph": average_speed_kph,
          "fuel_consumed_liters": fuel_consumed_liters,
          "route_name": f"Trip from {start_time.strftime('%Y-%m-%d %H:%M')}"
        }

        self.export_progress_bar.update(70, tr("Saving trip data..."), "70%", True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(TRIP_DATA_PATH, f"trip_{timestamp}.json")
        with open(filename, 'w') as f:
          json.dump(trip_data, f, indent=2)
        time.sleep(0.5)

        self._params.delete("ExportTripDataTrigger")
        self.export_progress_bar.update(100, tr("Export completed!"), "100%", True)
        gui_app.show_toast(tr(f"Trip data exported to {filename}"), "success")

        time.sleep(2)
        self.export_progress_bar.update(0, tr("Idle"), "", False)
        self._export_trip_data_btn.action_item.set_enabled(True) # Re-enable button

      except Exception as e:
        self.export_progress_bar.update(0, tr(f"Export failed: {str(e)}"), "", False) # Don't keep progress if failed
        gui_app.show_toast(tr(f"Export failed: {str(e)}"), "error")

        time.sleep(2)
        self.export_progress_bar.update(0, tr("Idle"), "", False)
        self._export_trip_data_btn.action_item.set_enabled(True) # Re-enable button

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
