#include "mainwin.h"
#include <iostream>

// Implementation of base UI element
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

// Implementation of main window
CabanaMainWindow::CabanaMainWindow() {
  // Initialize Raylib window
  InitWindow(1200, 800, "Cabana - Raylib Edition");
  SetTargetFPS(60);
  last_time = GetTime();
  
  createUIComponents();
}

CabanaMainWindow::~CabanaMainWindow() {
  CloseWindow();
}

void CabanaMainWindow::createUIComponents() {
  // Calculate initial UI layout
  float screen_width = 1200.0f;
  float screen_height = 800.0f;
  
  float menu_height = 30.0f;
  float status_height = 25.0f;
  float messages_width = 300.0f;
  float video_width = 600.0f;
  
  menu_bar_rect = {0, 0, screen_width, menu_height};
  messages_panel_rect = {0, menu_height, messages_width, screen_height - menu_height - status_height};
  video_panel_rect = {messages_width, menu_height, video_width, 300};
  charts_panel_rect = {messages_width, menu_height + 300, video_width, screen_height - menu_height - 300 - status_height};
  status_bar_rect = {0, screen_height - status_height, screen_width, status_height};
  
  // Create UI components
  messages_widget = std::make_unique<MessagesWidget>(this);
  video_widget = std::make_unique<VideoWidget>(this);
  charts_widget = std::make_unique<ChartsWidget>(this);
  center_widget = std::make_unique<CenterWidget>(this);
}

void CabanaMainWindow::run() {
  while (!WindowShouldClose() && running) {
    double current_time = GetTime();
    double dt = current_time - last_time;
    last_time = current_time;
    
    handleInput();
    update();
    render();
  }
}

void CabanaMainWindow::handleInput() {
  // Handle global input
  if (IsKeyPressed(KEY_F11)) {
    is_fullscreen = !is_fullscreen;
    ToggleFullscreen();
  }
  
  if (IsKeyPressed(KEY_F1)) {
    // Show help overlay
    // TODO: Implement help overlay functionality
  }
  
  // Handle input for all UI components
  if (show_messages_panel && messages_widget) {
    messages_widget->handleInput();
  }
  
  if (show_video_panel && video_widget) {
    video_widget->handleInput();
  }
  
  if (show_charts_panel && charts_widget) {
    charts_widget->handleInput();
  }
}

void CabanaMainWindow::update() {
  // Update UI components
  if (show_messages_panel && messages_widget) {
    messages_widget->update();
  }
  
  if (show_video_panel && video_widget) {
    video_widget->update();
  }
  
  if (show_charts_panel && charts_widget) {
    charts_widget->update();
  }
  
  // Update status message timeout
  if (!status_message.empty() && status_timeout > 0) {
    status_timeout -= GetFrameTime();
    if (status_timeout <= 0) {
      status_message.clear();
    }
  }
}

void CabanaMainWindow::render() {
  BeginDrawing();
  ClearBackground(RAYWHITE);
  
  // Render UI components
  if (show_messages_panel && messages_widget) {
    messages_widget->render(messages_panel_rect);
  }
  
  if (show_video_panel && video_widget) {
    video_widget->render(video_panel_rect);
  }
  
  if (show_charts_panel && charts_widget) {
    charts_widget->render(charts_panel_rect);
  }
  
  // Render menu bar
  DrawRectangleRec(menu_bar_rect, Color{220, 220, 220, 255});
  DrawRectangleLines(menu_bar_rect.x, menu_bar_rect.y, menu_bar_rect.width, menu_bar_rect.height, DARKGRAY);
  
  // Draw menu items
  const char* menuItems[] = {"File", "Edit", "View", "Tools", "Help"};
  int itemCount = sizeof(menuItems) / sizeof(menuItems[0]);
  float itemWidth = menu_bar_rect.width / itemCount;
  
  for (int i = 0; i < itemCount; ++i) {
    Rectangle itemRect = {menu_bar_rect.x + i * itemWidth, menu_bar_rect.y, itemWidth, menu_bar_rect.height};
    DrawText(menuItems[i], itemRect.x + 10, itemRect.y + 5, 14, DARKGRAY);
    
    // Draw separator line
    if (i < itemCount - 1) {
      DrawLineV({itemRect.x + itemWidth - 1, menu_bar_rect.y}, 
                {itemRect.x + itemWidth - 1, menu_bar_rect.y + menu_bar_rect.height}, LIGHTGRAY);
    }
  }
  
  // Render status bar
  DrawRectangleRec(status_bar_rect, Color{240, 240, 240, 255});
  DrawRectangleLines(status_bar_rect.x, status_bar_rect.y, 
                     status_bar_rect.width, status_bar_rect.height, LIGHTGRAY);
  
  std::string statusText = "Status: " + status_message + " | FPS: " + std::to_string(GetFPS());
  DrawText(statusText.c_str(), status_bar_rect.x + 5, status_bar_rect.y + 5, 12, DARKGRAY);
  
  EndDrawing();
}

void CabanaMainWindow::showMessage(const std::string &msg, int timeout) {
  status_message = msg;
  status_timeout = timeout / 1000.0f;  // Convert milliseconds to seconds
}