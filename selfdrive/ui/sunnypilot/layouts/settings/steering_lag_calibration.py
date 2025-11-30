import pyray as rl
from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.list_view import ListItem, Scroller, toggle_item_sp, option_item_sp, button_item
from openpilot.system.ui.widgets.widget import Widget


class SteeringLagCalibrationLayout(Widget):
  def __init__(self, parent: Widget):
    super().__init__(parent)
    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self) -> list[ListItem]:
    items = [
      toggle_item_sp(
        title=lambda: tr("Use Auto Steering Lag"),
        description=lambda: tr("When enabled, sunnypilot automatically learns and applies the optimal steering lag."),
        param="LagdToggle",
        icon="icons/sunnypilot.png"
      ),
      option_item_sp(
        title=lambda: tr("Manual Steering Lag Delay"),
        param="LagdToggleDelay",
        min_value=0,
        max_value=200, # Representing 0.0 to 2.0 seconds (e.g., 50 means 0.5s)
        description=lambda: tr("Set a manual steering lag delay when 'Use Auto Steering Lag' is disabled."),
        value_change_step=5,
        use_float_scaling=True, # Will scale value by 100 in the widget
        icon="icons/settings.png"
      ),
      button_item(
        title=lambda: tr("Reset Learned Steering Lag"),
        description=lambda: tr("Delete all automatically learned steering lag values."),
        callback=self._reset_learned_lag
      ),
    ]
    return items

  def _reset_learned_lag(self):
    self._params.delete("LiveDelay")
    self._params.delete("LiveTorqueParameters")
    gui_app.show_toast(tr("Learned steering lag reset successfully!"), "success")

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)
