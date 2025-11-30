"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class SteeringLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    from openpilot.system.ui.sunnypilot.widgets.list_view import button_item, toggle_item_sp
    from openpilot.system.ui.lib.multilang import tr

    items = [
      toggle_item_sp(
        title=lambda: tr("LKAS Toggle"),
        description=lambda: tr("Enable to turn on LKAS with a single tap on the gas pedal. Disable to use stock LKAS."),
        icon="openpilot/selfdrive/assets/icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Auto Lane Change"),
        description=lambda: tr("Enable to use sunnypilot's improved lane change assistant. No more need to hold blinker!"),
        param="AutoLaneChangeEnabled",
        icon="openpilot/selfdrive/assets/icons/sunnypilot.png"
      ),
      button_item(
        title=lambda: tr("LKAS Start Delay"),
        value=lambda: str(int(self._params.get("LKASStartDelay", encoding="utf-8"))),
        description=lambda: tr("Set the delay in seconds before LKAS engages after a tap on the gas pedal."),
        callback=self._set_lkas_start_delay
      ),
    ]
    return items

  def _set_lkas_start_delay(self):
    from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
    def on_callback(result, value):
      if result and value.isdigit():
        self._params.put("LKASStartDelay", str(int(value)))

    dialog = InputDialogSP(
      title="LKAS Start Delay",
      sub_title="Enter delay in seconds (0-10):",
      current_text=self._params.get("LKASStartDelay", encoding="utf-8") or "2",
      callback=on_callback,
      min_text_size=0
    )
    dialog.show()

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
