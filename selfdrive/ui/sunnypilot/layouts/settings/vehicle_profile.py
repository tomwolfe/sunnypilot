import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.list_view import ListItem, Scroller
from openpilot.system.ui.widgets.widget import Widget


class VehicleProfileLayout(Widget):
  def __init__(self, parent: Widget):
    super().__init__(parent)
    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self) -> list[ListItem]:
    from openpilot.system.ui.widgets.list_view import button_item, toggle_item_sp
    from openpilot.system.ui.lib.application import gui_app

    items = [
      button_item(
        title=lambda: tr("Create New Profile"),
        value=lambda: tr("CREATE"),
        description=lambda: tr("Create a new custom vehicle profile with specific settings."),
        callback=self._create_profile
      ),
      button_item(
        title=lambda: tr("Select Profile"),
        value=lambda: tr("SELECT"),
        description=lambda: tr("Choose an existing vehicle profile to activate."),
        callback=self._select_profile
      ),
      button_item(
        title=lambda: tr("Delete Profile"),
        value=lambda: tr("DELETE"),
        description=lambda: tr("Remove an existing vehicle profile."),
        callback=self._delete_profile
      ),
      toggle_item_sp(
        title=lambda: tr("Use Active Profile"),
        description=lambda: tr("Apply the currently selected vehicle profile settings."),
        param="UseCustomVehicleProfile",
        icon="icons/sunnypilot.png"
      ),
    ]
    return items

  def _create_profile(self):
    # This would open a dialog to create a new profile
    from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
    def on_callback(result, value):
      if result and value.strip():
        # Save the new profile name
        current_profiles = self._params.get("CustomVehicleProfiles", encoding="utf-8")
        if not current_profiles:
          current_profiles = value
        else:
          profiles_list = current_profiles.split(',')
          if value not in profiles_list:
            profiles_list.append(value)
            current_profiles = ','.join(profiles_list)
        self._params.put("CustomVehicleProfiles", current_profiles)
        self._params.put("ActiveVehicleProfile", value)
        from openpilot.system.ui.lib.application import gui_app
        gui_app.show_toast(tr("Profile created successfully!"), "success")

    dialog = InputDialogSP(
      title=tr("Create New Vehicle Profile"),
      sub_title=tr("Enter profile name:"),
      current_text="",
      callback=lambda result, value: on_callback(result, value),
      min_text_size=1
    )
    dialog.show()

  def _select_profile(self):
    from openpilot.system.ui.sunnypilot.widgets.selection_dialog import SelectionDialogSP
    profiles_str = self._params.get("CustomVehicleProfiles", encoding="utf-8")
    if profiles_str:
      profiles = [p.strip() for p in profiles_str.split(',') if p.strip()]
      if profiles:
        def on_callback(result, value):
          from openpilot.system.ui.lib.application import gui_app
          if result == DialogResult.CONFIRM and value:
            self._params.put("ActiveVehicleProfile", value)
            gui_app.show_toast(tr(f"Profile '{value}' selected!"), "success")
          elif result == DialogResult.CANCEL:
            gui_app.show_toast(tr("Profile selection cancelled."), "info")

        active_profile = self._params.get("ActiveVehicleProfile", encoding="utf-8") or ""
        dialog = SelectionDialogSP(
          title=tr("Select Vehicle Profile"),
          items=profiles,
          current_value=active_profile,
          callback=lambda result, value: on_callback(result, value)
        )
        dialog.show()
      else:
        from openpilot.system.ui.lib.application import gui_app
        gui_app.show_toast(tr("No profiles available to select!"), "error")
    else:
      from openpilot.system.ui.lib.application import gui_app
      gui_app.show_toast(tr("No profiles available!"), "error")

  def _delete_profile(self):
    from openpilot.system.ui.sunnypilot.widgets.selection_dialog import SelectionDialogSP
    profiles_str = self._params.get("CustomVehicleProfiles", encoding="utf-8")
    if profiles_str:
      profiles = [p.strip() for p in profiles_str.split(',') if p.strip()]
      if profiles:
        def on_callback(result, value):
          from openpilot.system.ui.lib.application import gui_app
          if result == DialogResult.CONFIRM and value:
            profiles.remove(value)
            if profiles:
              self._params.put("CustomVehicleProfiles", ','.join(profiles))
              active_profile = self._params.get("ActiveVehicleProfile", encoding="utf-8")
              if active_profile == value:
                self._params.delete("ActiveVehicleProfile")
            else:
              self._params.delete("CustomVehicleProfiles")
              self._params.delete("ActiveVehicleProfile")
            gui_app.show_toast(tr(f"Profile '{value}' deleted!"), "success")
          elif result == DialogResult.CANCEL:
            gui_app.show_toast(tr("Profile deletion cancelled."), "info")

        dialog = SelectionDialogSP(
          title=tr("Delete Vehicle Profile"),
          items=profiles,
          callback=lambda result, value: on_callback(result, value)
        )
        dialog.show()
      else:
        from openpilot.system.ui.lib.application import gui_app
        gui_app.show_toast(tr("No profiles available to delete!"), "error")
    else:
      from openpilot.system.ui.lib.application import gui_app
      gui_app.show_toast(tr("No profiles to delete!"), "error")

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)
