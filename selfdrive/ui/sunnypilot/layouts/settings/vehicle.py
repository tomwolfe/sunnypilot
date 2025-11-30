"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class VehicleLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    from openpilot.system.ui.widgets.list_view import toggle_item_sp, button_item
    from openpilot.common.params import Params
    from openpilot.system.ui.lib.multilang import tr

    items = [
      toggle_item_sp(
        title=lambda: tr("Custom Vehicle Model"),
        description=lambda: tr("Enable to use custom vehicle model parameters for improved control."),
        param="CustomVehicleModel",
        icon="icons/sunnypilot.png"
      ),
      toggle_item_sp(
        title=lambda: tr("Torque Vectoring"),
        description=lambda: tr("Enable advanced torque vectoring for better cornering and stability."),
        param="TorqueVectoring",
        icon="icons/sunnypilot.png"
      ),
      button_item(
        title=lambda: tr("Calibrate Vehicle"),
        value=lambda: tr("CALIBRATE"),
        description=lambda: tr("Run custom vehicle calibration procedure."),
        callback=self._calibrate_vehicle
      ),
      button_item(
        title=lambda: tr("Vehicle Profile"),
        value=lambda: tr("MANAGE"),
        description=lambda: tr("Manage custom vehicle profile settings."),
        callback=self._manage_vehicle_profile
      ),
    ]
    return items

  def _calibrate_vehicle(self):
    # This would start vehicle calibration
    pass

  def _manage_vehicle_profile(self):
    # This would open vehicle profile management
    pass

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
