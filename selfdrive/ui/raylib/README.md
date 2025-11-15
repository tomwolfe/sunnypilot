# Raylib UI Implementation for Sunnypilot

## Overview
This directory contains the complete Raylib-based UI implementation that replaces the previous Qt-based UI system. The transition to Raylib provides a more lightweight and efficient rendering solution.

## Architecture

### Core Components
- `raylib_ui_state.h/cc` - UI state management and data handling
- `raylib_ui_window.h/cc` - Main window management and UI element containers
- `raylib_ui_components.h/cc` - Onroad and offroad UI elements
- `raylib_settings_components.h/cc` - Settings UI elements
- `raylib_extra_components.h/cc` - Additional UI elements (body controls, driver view)
- `raylib_font_manager.h/cc` - Font loading and caching system
- `raylib_texture_manager.h/cc` - Texture loading and caching system
- `resource_manager.h/cc` - Resource cleanup and management

### UI State Management
The UI system uses a state-based approach where:
- `UIState` manages application state, messaging, and status updates
- `Device` manages device-specific functionality like brightness and wake state
- `UIScene` contains scene-specific data like camera textures and calibration

### UI Element Hierarchy
- `UIElement` - Base class for all UI elements
- `RaylibMainWindow` - Main container that manages different UI states
- Specific UI components inherit from UIElement and implement update/render methods

## Key Features Implemented

1. **Onroad UI** - Camera views, status indicators, alerts
2. **Offroad UI** - Home screen, visualizer, navigation buttons
3. **Settings UI** - Configurable panels with navigation sidebar
4. **Driver Monitoring UI** - Driver camera view with monitoring data
5. **Body Control UI** - Vehicle control interface (for generic robot mode)
6. **Sidebar Navigation** - Consistent navigation across states

## Building and Running
The UI is built automatically as part of the main sunnypilot build system. The SConscript file is configured to:
- Enable the USE_RAYLIB flag
- Include all necessary Raylib UI source files
- Link against the Raylib library
- Remove any Qt-related flags

## Camera Integration
The UI system supports multiple camera feeds:
- Road camera view for driving assistance
- Wide road camera view
- Driver monitoring camera view

Camera textures are managed through the UI state system and rendered in the appropriate UI elements.

## Color and Theming
The system uses the same status-based color scheme as the original Qt version:
- DISENGAGED: Blue-tinted background
- ENGADED: Green background
- OVERRIDE: Yellow background
- LAT_ONLY/LONG_ONLY: Special status colors

## Input Handling
- Mouse/touch input for UI interactions
- Click handling for buttons and navigation
- Proper callback system for UI events

## Performance Considerations
- Target 20 FPS rendering rate (matching UI_FREQ constant)
- Texture caching to reduce load times
- Efficient rendering pipeline
- Resource management to prevent memory leaks

## Testing and Validation
The implementation has been tested to ensure:
- All UI states render correctly
- State transitions work as expected
- Camera feeds display properly
- Settings navigation functions correctly
- Proper resource cleanup on exit

## Migration from Qt
This implementation completely replaces the Qt-based UI system with no remaining Qt dependencies in the main UI codebase. The original Qt code has been removed and the build system has been updated accordingly.

## Future Enhancements
- Enhanced camera texture rendering
- More sophisticated UI animations
- Performance optimizations
- Additional UI themes and customization options