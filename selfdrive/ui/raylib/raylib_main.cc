#include <sys/resource.h>
#include <raylib.h>

#include "system/hardware/hw.h"
#include "selfdrive/ui/raylib/raylib_window.h"
#include "selfdrive/ui/raylib/raylib_ui_state.h"

// For simplicity, using the same initialization
void initApp_raylib(int argc, char *argv[]);  // Assuming this exists elsewhere

int main_raylib(int argc, char *argv[]) {
  setpriority(PRIO_PROCESS, 0, -20);

  // Initialize Raylib window
  set_main_window_raylib(nullptr);
  
  // Initialize UI State
  UIState *ui_state = uiState_raylib();
  Device *device = device_raylib();
  
  // Main render loop
  while (!WindowShouldClose()) {
    // Update UI state
    ui_state->update();
    device->update(*ui_state);
    
    // Begin drawing
    BeginDrawing();
    
    // Clear screen with UI background color based on status
    ClearBackground(bg_colors[ui_state->status].to_raylib_color());
    
    // TODO: Add actual UI rendering here based on status and scene
    
    // End drawing
    EndDrawing();
  }
  
  // Cleanup
  CloseWindow();
  delete ui_state->prime_state;
  
  return 0;
}