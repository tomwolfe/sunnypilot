"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
import os
import json
from datetime import datetime

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.list_view import ListItem, Scroller
from openpilot.system.ui.widgets.widget import Widget

from openpilot.common.constants import TRIP_DATA_PATH


class TripStatisticsLayout(Widget):
  def __init__(self, parent: Widget):
    super().__init__(parent)
    self._params = Params()
    self.trip_data = [] # Initialize trip_data
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self) -> list[ListItem]:
    from openpilot.system.ui.widgets.list_view import button_item
    
    items = []

    if not self.trip_data:
      items.append(
        ListItem(
          title=lambda: tr("No Trip Statistics Available"),
          description=lambda: tr("Export some trip data first from the Trips menu."),
        )
      )
    else:
      for i, trip in enumerate(self.trip_data):
        start_time_str = datetime.fromisoformat(trip["start_time"]).strftime("%Y-%m-%d %H:%M")
        end_time_str = datetime.fromisoformat(trip["end_time"]).strftime("%H:%M")
        title = f"{start_time_str} - {end_time_str}"
        description = f"Dist: {trip['distance_meters']/1000:.1f} km, Avg Speed: {trip['average_speed_kph']:.1f} km/h, Fuel: {trip['fuel_consumed_liters']:.1f} L"
        items.append(
          ListItem(
            title=lambda: tr(title),
            description=lambda: tr(description),
          )
        )
    
    items.append(
      button_item(
        title=lambda: tr("Refresh Statistics"),
        value=lambda: tr("REFRESH"),
        description=lambda: tr("Update trip statistics from stored data."),
        callback=self._refresh_stats
      )
    )
    return items

  def _refresh_stats(self):
    gui_app.show_toast(tr("Refreshing trip statistics..."), "info")
    self.trip_data = []
    if os.path.exists(TRIP_DATA_PATH):
      for filename in os.listdir(TRIP_DATA_PATH):
        if filename.endswith(".json"):
          filepath = os.path.join(TRIP_DATA_PATH, filename)
          try:
            with open(filepath, 'r') as f:
              trip = json.load(f)
              self.trip_data.append(trip)
          except json.JSONDecodeError as e:
            print(f"Error decoding JSON from {filepath}: {e}")
            gui_app.show_toast(tr(f"Error reading trip file: {filename}"), "error")
          except Exception as e:
            print(f"Error reading {filepath}: {e}")
            gui_app.show_toast(tr(f"Error reading trip file: {filename}"), "error")
    
    # Sort trips by start time, newest first
    self.trip_data.sort(key=lambda x: datetime.fromisoformat(x['start_time']), reverse=True)
    
    # Update scroller items to display new data
    self._scroller.set_items(self._initialize_items()) # Assuming Scroller has a set_items method
    gui_app.show_toast(tr(f"Loaded {len(self.trip_data)} trips."), "success")

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)