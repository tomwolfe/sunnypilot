#include <raylib.h>
#include <iostream>
#include <memory>
#include <string>

#include "cabana_ui_components.h"

// Main entry point for cabana using Raylib
int main(int argc, char *argv[]) {
  std::cout << "Starting Cabana with Raylib UI..." << std::endl;

  // Initialize Raylib window
  InitWindow(1200, 800, "Cabana - Raylib Edition");
  SetTargetFPS(60);

  // Create and run the main window
  CabanaMainWindow mainWindow;

  // Main event loop
  while (!WindowShouldClose()) {
    mainWindow.handleInput();
    mainWindow.update();
    mainWindow.render();
  }

  // Cleanup
  CloseWindow();
  std::cout << "Cabana closed." << std::endl;
  return 0;
}