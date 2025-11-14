#include "selfdrive/ui/raylib/raylib_window.h"
#include "common/util.h"
#include "system/hardware/hw.h"

void set_main_window_raylib(void *w) {
  const float scale = util::getenv("SCALE", 1.0f);
  
  // Initialize Raylib window
  const int width = 2160;  // DEVICE_SCREEN_WIDTH * scale
  const int height = 1080; // DEVICE_SCREEN_HEIGHT * scale
  
  InitWindow(width, height, "Sunnypilot UI");
  
  // Set fullscreen for device
  if (!Hardware::PC()) {
    SetWindowState(FLAG_FULLSCREEN_MODE);
  }
  
  SetTargetFPS(60); // Set desired FPS
}

// Clean up function
void cleanup_window_raylib() {
  CloseWindow();
}