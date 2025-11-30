"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.list_view import ListItem, Scroller
from openpilot.system.ui.widgets.widget import Widget


class TripStatisticsLayout(Widget):
  def __init__(self, parent: Widget):
    super().__init__(parent)
    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self) -> list[ListItem]:
    from openpilot.system.ui.widgets.list_view import button_item
    
    items = [
      ListItem(
        title=lambda: tr("Trip Statistics"),
        description=lambda: tr("Detailed statistics about your trips will be displayed here."),
      ),
      button_item(
        title=lambda: tr("Refresh Statistics"),
        value=lambda: tr("REFRESH"),
        description=lambda: tr("Update trip statistics from stored data."),
        callback=self._refresh_stats
      ),
    ]
    return items

  def _refresh_stats(self):
    # In a real implementation, this would refresh the trip statistics
    gui_app.show_toast(tr("Refreshing trip statistics..."), "info")

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)