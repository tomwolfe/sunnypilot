import json
import pyray as rl
import jsonschema # Added import

from openpilot.common.params import Params
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.list_view import ListItem, Scroller
from openpilot.system.ui.widgets.widget import Widget

VEHICLE_PROFILE_SCHEMA = {
    "type": "object",
    "properties": {
        "torque_level": {"type": "integer", "minimum": 0},
        "steering_ratio": {"type": "number", "exclusiveMinimum": 0},
    },
    "required": ["torque_level", "steering_ratio"],
    "additionalProperties": True
}


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
      button_item(
        title=lambda: tr("Edit Current Profile Settings"),
        value=lambda: tr("EDIT"),
        description=lambda: tr("Modify settings for the currently active vehicle profile."),
        callback=self._edit_profile_settings
      ),
    ]
    return items


  def _save_profile_settings(self, profile_name: str, settings: dict):
    key = f"VehicleProfile_{profile_name.replace(' ', '_')}_SETTINGS"
    self._params.put(key, json.dumps(settings))

  def _load_profile_settings(self, profile_name: str) -> dict:
    key = f"VehicleProfile_{profile_name.replace(' ', '_')}_SETTINGS"
    settings_str = self._params.get(key, encoding="utf-8")
    if settings_str:
      return json.loads(settings_str)
    return {} # Return empty dict if no settings found

  def _create_profile(self):
    # This would open a dialog to create a new profile
    from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
    def on_callback(result, value):
      if result and value.strip():
        profile_name = value.strip()
        # Save the new profile name
        current_profiles = self._params.get("CustomVehicleProfiles", encoding="utf-8")
        profiles_list = []
        if current_profiles:
          profiles_list = [p.strip() for p in current_profiles.split(',') if p.strip()]

        if profile_name not in profiles_list:
          profiles_list.append(profile_name)
          self._params.put("CustomVehicleProfiles", ','.join(profiles_list))
          self._params.put("ActiveVehicleProfile", profile_name)

          # Initialize settings for the new profile
          default_settings = {
              "torque_level": 100, # Example default setting
              "steering_ratio": 15.0, # Example default setting
              # Add other default settings here
          }
          self._save_profile_settings(profile_name, default_settings)
          from openpilot.system.ui.lib.application import gui_app
          gui_app.show_toast(tr("Profile created and activated successfully!"), "success")
        else:
          from openpilot.system.ui.lib.application import gui_app
          gui_app.show_toast(tr("Profile name already exists!"), "error")

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
            # Load settings for the newly selected profile
            settings = self._load_profile_settings(value)
            gui_app.show_toast(tr(f"Profile '{value}' selected and settings loaded!"), "success")
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
              if active_profile == value: # If the deleted profile was active
                self._params.put("ActiveVehicleProfile", profiles[0]) # Set the first remaining profile as active
            else:
              self._params.delete("CustomVehicleProfiles")
              self._params.delete("ActiveVehicleProfile")
            gui_app.show_toast(tr(f"Profile '{value}' deleted!"), "success")
            # Delete associated settings
            settings_key = f"VehicleProfile_{value.replace(' ', '_')}_SETTINGS"
            self._params.delete(settings_key)
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

  def _edit_profile_settings(self):
    from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
    from openpilot.system.ui.lib.application import gui_app

    active_profile = self._params.get("ActiveVehicleProfile", encoding="utf-8")
    if not active_profile:
      gui_app.show_toast(tr("No active profile to edit settings for!"), "error")
      return

    current_settings = self._load_profile_settings(active_profile)
    current_settings_json = json.dumps(current_settings, indent=2)

    def on_callback(result, value):
      if result and value.strip():
        try:
          new_settings = json.loads(value)
          jsonschema.validate(instance=new_settings, schema=VEHICLE_PROFILE_SCHEMA) # Validate here
          self._save_profile_settings(active_profile, new_settings)
          gui_app.show_toast(tr("Profile settings updated successfully!"), "success")
        except json.JSONDecodeError:
          gui_app.show_toast(tr("Invalid JSON format! Please correct and try again."), "error")
        except jsonschema.ValidationError as e:
          gui_app.show_toast(tr(f"Profile validation error: {e.message}"), "error") # Show validation error
      elif result == DialogResult.CANCEL:
        gui_app.show_toast(tr("Editing profile settings cancelled."), "info")

    dialog = InputDialogSP(
      title=tr(f"Edit Settings for '{active_profile}'"),
      sub_title=tr("Edit JSON settings:"),
      current_text=current_settings_json,
      callback=lambda result, value: on_callback(result, value),
      min_text_size=1
    )
    dialog.show()

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)
