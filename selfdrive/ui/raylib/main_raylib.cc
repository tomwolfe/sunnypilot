#include <sys/resource.h>
#include <raylib.h>

#include "system/hardware/hw.h"
#include "selfdrive/ui/raylib/raylib_window.h"
#include "selfdrive/ui/raylib/raylib_ui_state.h"
#include "selfdrive/ui/raylib/raylib_ui_window.h"
#include "common/params.h"
#include "common/swaglog.h"

// Initialize the application (this would include logging setup, etc.)
void initApp_raylib(int argc, char *argv[]) {
  // In the real implementation, this would do the same initialization as the Qt version
  // For now, we'll just set up basic parameters
  Params().initialize();
}

int main(int argc, char *argv[]) {
  setpriority(PRIO_PROCESS, 0, -20);

  // Initialize Raylib window
  set_main_window_raylib(nullptr);
  
  // Initialize application
  initApp_raylib(argc, argv);
  
  // Initialize UI State
  UIState *ui_state = uiState_raylib();
  Device *device = device_raylib();
  
  // Create main window
  RaylibMainWindow main_window;
  
  // Set up callbacks to handle UI state changes
  ui_state->uiUpdateCallback = [&](const UIState &s) {
    main_window.update(s);
  };
  
  ui_state->offroadTransitionCallback = [&](bool offroad) {
    if (!offroad) {
      main_window.closeSettings();  // Close settings when going onroad
    }
  };
  
  ui_state->engagedChangedCallback = [&](bool engaged) {
    // Handle engaged state changes
  };
  
  device->interactiveTimeoutCallback = [&]() {
    // Close settings when interactive timeout occurs
    main_window.closeSettings();
  };
  
  // Main render loop
  SetTargetFPS(20); // Match UI_FREQ of 20Hz
  while (!WindowShouldClose()) {
    // Update device state
    device->update(*ui_state);
    
    // Update UI state - this will trigger callbacks and UI updates
    ui_state->update();
    
    // Begin drawing
    BeginDrawing();
    
    // Render the main UI
    main_window.render(*ui_state);
    
    // End drawing
    EndDrawing();
  }
  
  // Cleanup
  CloseWindow();
  delete ui_state->prime_state;
  
  return 0;
}