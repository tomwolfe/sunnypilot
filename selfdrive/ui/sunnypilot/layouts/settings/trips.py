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
    self._params = Params()
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
    print("Trip statistics feature under development.")
    # TODO: Implement navigation to a new UI screen for trip statistics

  def _export_trip_data(self):
    self._params.put_bool("ExportTripDataTrigger", True)
    self.export_progress_bar.update(0, tr("Exporting..."), True)

  def update(self):
    # Check for export status
    if self._params.get_bool("ExportTripDataTrigger"):
      self.export_progress_bar.update(0, tr("Exporting..."), True)
    else:
      self.export_progress_bar.update(0, tr("Idle"), False)


  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
