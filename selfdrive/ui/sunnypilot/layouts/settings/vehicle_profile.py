import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.list_view import ListItem, Scroller
from openpilot.system.ui.widgets.widget import Widget


class VehicleProfileLayout(Widget):
  def __init__(self, parent: Widget):
    super().__init__(parent)
    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self) -> list[ListItem]:
    items = [
      # Placeholder for future vehicle profile management UI elements
      ListItem(
        title=lambda: tr("Custom Vehicle Profiles"),
        description=lambda: tr("This feature is under development. Here you will be able to manage custom vehicle specific settings."),
      ),
      # Add more UI elements here as the feature develops
    ]
    return items

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)
