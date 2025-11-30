"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class VisualsLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    from openpilot.system.ui.widgets.list_view import toggle_item_sp, option_item_sp
    from openpilot.common.params import Params
    from openpilot.system.ui.lib.multilang import tr

    items = [
      toggle_item_sp(
        title=lambda: tr("Show Speed"),
        description=lambda: tr("Display current speed in the onroad UI. Toggle to show or hide."),
        param="ShowSpeed",
        icon="icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Show Speed Limit"),
        description=lambda: tr("Display speed limit on the road. Adjusts based on map data and speed limit signs."),
        param="ShowSpeedLimit",
        icon="icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Show ETA"),
        description=lambda: tr("Show estimated time of arrival on the navigation screen."),
        param="ShowETA",
        icon="icons/sunnypilot.png"
      ),
      option_item_sp(
        title=lambda: tr("Brightness"),
        param="Brightness",
        min_value=50,
        max_value=100,
        description=lambda: tr("Adjust display brightness percentage."),
        value_change_step=5,
        icon="icons/settings.png"
      ),
    ]
    return items

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
