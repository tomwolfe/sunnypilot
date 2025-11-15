#include "cabana_ui_components.h"
#include "cabana_panels.h"
#include <raylib.h>

// Base UI element implementation
CabanaUIElement::CabanaUIElement(float x, float y, float width, float height)
  : bounds({x, y, width, height}), isVisible(true) {
}

void CabanaUIElement::setPosition(float x, float y) {
  bounds.x = x;
  bounds.y = y;
}

void CabanaUIElement::setSize(float width, float height) {
  bounds.width = width;
  bounds.height = height;
}

void CabanaUIElement::handleInput() {
  // Default implementation - can be overridden by subclasses
}

// Implementation of the main UI window
CabanaMainWindow::CabanaMainWindow() {
  // Initialize UI elements with screen dimensions
  float screen_width = GetScreenWidth();
  float screen_height = GetScreenHeight();

  // Create UI components with proper layout
  float menu_height = 30;
  float status_height = 25;
  float messages_width = 300;
  float video_width = 600;

  menuBar = std::make_unique<MenuBar>(0, 0, screen_width, menu_height);
  messagesPanel = std::make_unique<MessagesPanel>(0, menu_height, messages_width, screen_height - menu_height - status_height);
  videoPanel = std::make_unique<VideoPanel>(messages_width, menu_height, video_width, 300);
  chartsPanel = std::make_unique<ChartsPanel>(messages_width, menu_height + 300, video_width, screen_height - menu_height - 300 - status_height);
  statusBar = std::make_unique<StatusBar>(0, screen_height - status_height, screen_width, status_height);
}

void CabanaMainWindow::update() {
  // Update all UI elements
  if (menuBar && menuBar->getVisible()) {
    menuBar->update();
  }
  if (messagesPanel && messagesPanel->getVisible()) {
    messagesPanel->update();
  }
  if (videoPanel && videoPanel->getVisible()) {
    videoPanel->update();
  }
  if (chartsPanel && chartsPanel->getVisible()) {
    chartsPanel->update();
  }
  if (statusBar && statusBar->getVisible()) {
    // Update status bar
    statusBar->update();
  }
}

void CabanaMainWindow::render() {
  BeginDrawing();
  ClearBackground(RAYWHITE);

  // Render all UI elements
  if (menuBar && menuBar->getVisible()) {
    menuBar->render();
  }
  if (messagesPanel && messagesPanel->getVisible()) {
    messagesPanel->render();
  }
  if (videoPanel && videoPanel->getVisible()) {
    videoPanel->render();
  }
  if (chartsPanel && chartsPanel->getVisible()) {
    chartsPanel->render();
  }
  if (statusBar && statusBar->getVisible()) {
    // Render status bar background
    DrawRectangleRec(statusBar->getBounds(), Color{240, 240, 240, 255});
    DrawRectangleLines(statusBar->getBounds().x, statusBar->getBounds().y,
                      statusBar->getBounds().width, statusBar->getBounds().height, LIGHTGRAY);
    DrawText("Status: Ready | FPS: 60", statusBar->getBounds().x + 5,
             statusBar->getBounds().y + 5, 12, DARKGRAY);
  }

  EndDrawing();
}

void CabanaMainWindow::handleInput() {
  if (WindowShouldClose()) {
    running = false;
  }
}

// Basic rendering functions
void drawBackground(Color color) {
  ClearBackground(color);
}

void drawRect(const Rectangle& rect, Color color) {
  DrawRectangleRec(rect, color);
}

void drawText(const char* text, float x, float y, int fontSize, Color color) {
  DrawText(text, (int)x, (int)y, fontSize, color);
}

void drawTextCentered(const char* text, Rectangle container, int fontSize, Color color) {
  int textWidth = MeasureText(text, fontSize);
  float x = container.x + (container.width - textWidth) / 2.0f;
  float y = container.y + (container.height - fontSize) / 2.0f;
  DrawText(text, (int)x, (int)y, fontSize, color);
}

// Implementation of StatusBar methods
StatusBar::StatusBar(float x, float y, float width, float height)
  : CabanaUIElement(x, y, width, height) {
}

void StatusBar::update() {
  // Update status bar state if needed
  // For example, update FPS counter
  fps = GetFPS();
}

void StatusBar::render() {
  if (!isVisible) return;

  // Render status bar background
  DrawRectangleRec(bounds, Color{240, 240, 240, 255});
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);

  // Create status text
  char statusStr[256];
  snprintf(statusStr, sizeof(statusStr), "Status: %s | FPS: %d", statusText.c_str(), fps);

  DrawText(statusStr, bounds.x + 5, bounds.y + 5, 12, DARKGRAY);
}