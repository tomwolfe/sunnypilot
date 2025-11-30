"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class OSMLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, button_item
    from openpilot.system.ui.lib.multilang import tr

    items = [
      toggle_item_sp(
        title=lambda: tr("OSM Integration"),
        description=lambda: tr("Enable OpenStreetMap integration for enhanced navigation and speed limit data."),
        param="OSMEnabled",
        icon="icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Speed Limit Sign Recognition"),
        description=lambda: tr("Enable to use camera-based speed limit sign recognition in addition to map data."),
        param="SLSREnabled",
        icon="icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Map Scale to Speed"),
        description=lambda: tr("Automatically adjust map scale based on vehicle speed for better visibility."),
        param="MapScaleToSpeed",
        icon="icons/sunnypilot.png"
      ),
      button_item(
        title=lambda: tr("Map Update"),
        value=lambda: tr("UPDATE"),
        description=lambda: tr("Download the latest map data from OpenStreetMap."),
        callback=self._update_map_data
      ),
    ]
    return items

  def _update_map_data(self):
    # This would be connected to a map update routine
    self._params.put("LastMapUpdateRequest", "1")

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
