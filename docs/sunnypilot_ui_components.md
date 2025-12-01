# Sunnypilot UI Components Documentation

This document describes the new UI components added to sunnypilot and how they are implemented.

## OptionControlSP

The `OptionControlSP` is a UI component that allows users to select numeric values within a range using '+' and '-' buttons.

### Implementation Details
- Located in: `system/ui/sunnypilot/widgets/option_control.py`
- Extends: `ItemAction`
- Purpose: Provides a user-friendly way to adjust numeric parameters
- Features:
  - Minus button to decrease value
  - Plus button to increase value
  - Visual display of current value
  - Bounds checking (min/max values)
  - Parameter persistence

### Usage Example
```python
from openpilot.system.ui.sunnypilot.widgets.option_control import OptionControlSP

# Create an option control for UI Scale
option_control = OptionControlSP(
    param="UIScale",
    min_value=70,
    max_value=130,
    value_change_step=5,
    enabled=True
)
```

## InputDialogSP

The `InputDialogSP` is a text input dialog for collecting user input.

### Implementation Details
- Located in: `system/ui/sunnypilot/widgets/input_dialog.py`
- Purpose: Collect text input from users
- Features:
  - On-screen keyboard integration
  - Text validation
  - Optional parameter saving
  - Modal overlay display

### Usage Example
```python
from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP

def on_callback(result, value):
    if result and value.lstrip('-').isdigit():
        # Process the input value
        print(f"User entered: {value}")

dialog = InputDialogSP(
    title="Cruise Speed Limit Offset",
    sub_title="Enter offset in mph (-25 to 25):",
    current_text="0",
    callback=on_callback,
    min_text_size=0
)
dialog.show()
```

## ProgressBarAction

The `ProgressBarAction` shows progress for long-running tasks.

### Implementation Details
- Located in: `system/ui/sunnypilot/widgets/progress_bar.py`
- Purpose: Visual feedback for operations with progress tracking
- Features:
  - Progress percentage display
  - Customizable text display
  - Visual progress bar

## FuzzySearch Helper

The `fuzzy_search.py` provides text search capabilities with fuzzy matching.

### Implementation Details
- Located in: `system/ui/sunnypilot/widgets/helpers/fuzzy_search.py`
- Purpose: Enable intelligent text filtering and search
- Features:
  - Normalizes text for comparison
  - Handles special characters and accents
  - Matches partial strings

## UI List View Extensions

The `list_view.py` file includes new functions for creating UI elements:

### option_item_sp
Creates an option control item for the settings list view.

### toggle_item_sp
Creates a toggle switch item for boolean parameters.

### button_item
Creates a button item that can trigger callbacks.

## Integration with Settings Layouts

These components are integrated into the various settings layouts:

- `cruise.py` - Cruise control settings
- `display.py` - Display settings
- `steering.py` - Steering settings
- `osm.py` - OSM integration settings
- `trips.py` - Trip logging settings
- `vehicle.py` - Vehicle settings
- `visuals.py` - Visual display settings