"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class CruiseLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, button_item
    from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
    from openpilot.system.ui.lib.multilang import tr

    items = [
      toggle_item_sp(
        title=lambda: tr("Cruise Stand Still"),
        description=lambda: tr("Enable to allow cruise to come to a complete stop and resume automatically. Disable to use stock cruise."),
        param="CruiseStandStill",
        icon="icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Smart Cruise Control"),
        description=lambda: tr("Enable sunnypilot's smart cruise that automatically adjusts to traffic. Works with stock ACC."),
        param="SmartCruiseEnabled",
        icon="icons/sunnypilot.png"
      ),
      button_item(
        title=lambda: tr("Cruise Speed Limit Offset"),
        value=lambda: str(int(self._params.get("CruiseSpeedLimitOffset", encoding="utf-8"))),
        description=lambda: tr("Set the offset in mph/km/h from the speed limit that you are comfortable with."),
        callback=self._set_cruise_speed_offset
      ),
    ]
    return items

  def _set_cruise_speed_offset(self):
    def on_callback(result, value):
      if result and value.lstrip('-').isdigit():
        self._params.put("CruiseSpeedLimitOffset", str(int(value)))

    from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
    dialog = InputDialogSP(
      title="Cruise Speed Limit Offset",
      sub_title="Enter offset in mph (-25 to 25):",
      current_text=self._params.get("CruiseSpeedLimitOffset", encoding="utf-8") or "0",
      callback=lambda result, value: on_callback(result, value),
      min_text_size=0
    )
    dialog.show()

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
