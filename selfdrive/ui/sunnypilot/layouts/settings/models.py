"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item
from openpilot.system.ui.lib.multilang import tr

# Constants
DESCRIPTION = ("User-settable factor (0.0 to 1.0) to simulate thermal conditions and fine-tune "
               "model execution performance. Values below 1.0 will throttle model inference frequency.")
THROTTLE_FACTORS = [0.0, 0.25, 0.5, 0.75, 1.0]
BUTTON_TEXTS = ["0%", "25%", "50%", "75%", "100%"] # Representing percentage of full execution


class ModelsLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    self._model_throttle_factor_setting = multiple_button_item(
      tr("Model Execution Throttle Factor"),
      tr(DESCRIPTION),
      buttons=[tr(text) for text in BUTTON_TEXTS],
      button_width=180,
      callback=self._set_model_throttle_factor,
      selected_index=self._get_current_throttle_factor_index(),
      icon="settings.png"  # Placeholder icon, consider a more appropriate one
    )

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    items = [
      self._model_throttle_factor_setting,
    ]
    return items

  def _get_current_throttle_factor_index(self) -> int:
    val = self._params.get("ModelExecutionThrottleFactor", return_default=True)
    current_value = float(val) if val is not None else 1.0
    # Find the index of the closest value. If not exact, default to 1.0 (100%)
    try:
      return THROTTLE_FACTORS.index(current_value)
    except ValueError:
      return THROTTLE_FACTORS.index(1.0) # Default to 100% if current value is not in discrete options

  def _set_model_throttle_factor(self, index: int):
    value = THROTTLE_FACTORS[index]
    self._params.put("ModelExecutionThrottleFactor", str(value))

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
