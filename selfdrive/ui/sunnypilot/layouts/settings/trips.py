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

from openpilot.common.threading_util import start_background_task
from openpilot.common.params import Params
from openpilot.common.constants import TRIP_DATA_PATH, TRIP_DATA_RETENTION_COUNT_PARAM_KEY, DEFAULT_TRIP_DATA_RETENTION_COUNT

from openpilot.selfdrive.ui.sunnypilot.lib.trip_data_collector import trip_data_collector


class TripsLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)
    self._export_lock = threading.Lock()

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

  def _cleanup_trip_data(self):
    from openpilot.system.ui.lib.application import gui_app
    try:
      retention_count = int(self._params.get(TRIP_DATA_RETENTION_COUNT_PARAM_KEY, encoding='utf-8') or DEFAULT_TRIP_DATA_RETENTION_COUNT)
    except (ValueError, TypeError):
      retention_count = DEFAULT_TRIP_DATA_RETENTION_COUNT # Fallback to default if param is invalid

    if not os.path.exists(TRIP_DATA_PATH):
      return

    all_trip_files = []
    for filename in os.listdir(TRIP_DATA_PATH):
      if filename.startswith("trip_") and filename.endswith(".json"):
        filepath = os.path.join(TRIP_DATA_PATH, filename)
        if os.path.isfile(filepath):
          all_trip_files.append((filepath, os.path.getmtime(filepath)))

    # Sort files by modification time, oldest first
    all_trip_files.sort(key=lambda x: x[1])

    files_to_delete = all_trip_files[:-retention_count] # Keep the newest 'retention_count' files

    for filepath, _ in files_to_delete:
      try:
        os.remove(filepath)
        print(f"Cleaned up old trip data file: {filepath}")
      except OSError as e:
        print(f"Error deleting file {filepath}: {e}")
        gui_app.show_toast(tr(f"Error cleaning up old trip data: {os.path.basename(filepath)}"), "error")

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
      if not self._export_lock.acquire(blocking=False):
        gui_app.show_toast(tr("Another export is already in progress."), "info")
        # Ensure the button is re-enabled if we return early due to existing export
        self._export_trip_data_btn.action_item.set_enabled(True)
        self.export_progress_bar.update(0, tr("Idle"), "", False) # Reset progress bar
        return

      try:
        self._export_trip_data_btn.action_item.set_enabled(False) # Disable button during export
        self.export_progress_bar.update(5, tr("Preparing export..."), "5%", True)
        os.makedirs(TRIP_DATA_PATH, exist_ok=True)

        trip_data_from_collector = trip_data_collector.get_trip_data()

        if trip_data_from_collector is None or trip_data_from_collector.get("is_active"):
          gui_app.show_toast(tr("No completed trip data available to export."), "warning")
          self._export_trip_data_btn.action_item.set_enabled(True)
          self.export_progress_bar.update(0, tr("Idle"), "", False)
          if self._export_lock.locked():
            self._export_lock.release()
          return

        start_time = trip_data_from_collector["start_time"]
        end_time = trip_data_from_collector["end_time"]
        duration = trip_data_from_collector["duration_seconds"]
        distance_meters = trip_data_from_collector["distance_meters"]
        average_speed_kph = trip_data_from_collector["average_speed_kph"]
        fuel_consumed_percentage = trip_data_from_collector["fuel_consumed_percentage"]
        route_geojson = trip_data_from_collector["route_geojson"]
        vin = trip_data_from_collector["vin"]
        car_model = trip_data_from_collector["car_model"]

        # Get fuel tank capacity from parameter, with fallback to default if not set
        try:
          tank_capacity_param = self._params.get(FUEL_TANK_CAPACITY_PARAM_KEY, encoding='utf-8')
          if tank_capacity_param:
            tank_capacity_liters = float(tank_capacity_param)
          else:
            tank_capacity_liters = DEFAULT_FUEL_TANK_CAPACITY
        except (ValueError, TypeError):
          tank_capacity_liters = DEFAULT_FUEL_TANK_CAPACITY # Fallback to default if param value is invalid
        # If fuel consumption percentage is -1, it means it couldn't be calculated
        if fuel_consumed_percentage == -1:
          fuel_consumed_liters = -1  # Use -1 to indicate unknown fuel consumption
        else:
          fuel_consumed_liters = round(tank_capacity_liters * (fuel_consumed_percentage / 100.0), 2)

        route_name = self._params.get("CurrentRoute", encoding='utf8') # This might still be useful



        trip_data = {
          "start_time": start_time,
          "end_time": end_time,
          "duration_seconds": round(duration, 2),
          "distance_meters": distance_meters,
          "average_speed_kph": average_speed_kph,
          "fuel_consumed_liters": fuel_consumed_liters,
          "fuel_consumed_percentage": fuel_consumed_percentage,
          "route_geojson": route_geojson,
          "route_name": route_name if route_name else "N/A",
          "vin": vin,
          "car_model": car_model
        }

        self.export_progress_bar.update(70, tr("Saving trip data..."), "70%", True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(TRIP_DATA_PATH, f"trip_{timestamp}.json")
        with open(filename, 'w') as f:
          json.dump(trip_data, f, indent=2)


        self.export_progress_bar.update(100, tr("Export completed!"), "100%", True)
        gui_app.show_toast(tr(f"Trip data exported to {filename}"), "success")

        self._cleanup_trip_data() # Call cleanup after successful export


        self.export_progress_bar.update(0, tr("Idle"), "", False)
        self._export_trip_data_btn.action_item.set_enabled(True) # Re-enable button

      except Exception as e:
        self.export_progress_bar.update(0, tr(f"Export failed: {str(e)}"), "", False) # Don't keep progress if failed
        gui_app.show_toast(tr(f"Export failed: {str(e)}"), "error")

        self.export_progress_bar.update(0, tr("Idle"), "", False)
        self._export_trip_data_btn.action_item.set_enabled(True) # Re-enable button
      finally:
        if self._export_lock.locked():
          self._export_lock.release()

    start_background_task(export_trip_data_process, name="export_trip_data_process")



  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
