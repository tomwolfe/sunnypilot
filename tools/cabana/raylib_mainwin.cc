#include "tools/cabana/raylib_mainwin.h"
#include "tools/cabana/raylib_videowidget.h"
#include "tools/cabana/raylib_messageswidget.h"
#include "tools/cabana/raylib_chartswidget.h"
#include "tools/cabana/raylib_centerwidget.h"
#include "tools/cabana/streams/abstractstream.h"
#include "tools/cabana/utils/logger.h"

#include <iostream>

extern AbstractStream *can; // Global stream pointer

CabanaMainWindow::CabanaMainWindow() {
  Logger::instance().info("MAIN_WINDOW", "Initializing CabanaMainWindow");
  // Initialize Raylib window components
  InitWindow(1200, 800, "Cabana - Raylib Version");
  SetTargetFPS(60);

  // Initialize UI components
  createUIComponents();

  // Connect UI components to stream if available
  if (can) {
    Logger::instance().info("MAIN_WINDOW", "Connecting UI components to stream");
    if (messages_widget) {
      messages_widget->connectToStream(can);
    }
    // Connect other widgets to stream as needed
  } else {
    Logger::instance().warning("MAIN_WINDOW", "No stream available to connect to");
  }
  Logger::instance().info("MAIN_WINDOW", "CabanaMainWindow initialized successfully");
}

CabanaMainWindow::~CabanaMainWindow() {
  Logger::instance().info("MAIN_WINDOW", "Destroying CabanaMainWindow");
  // Disconnect widgets from stream
  if (messages_widget) {
    messages_widget->disconnectFromStream();
  }
  // Disconnect other widgets

  // Cleanup UI components
  if (IsWindowReady()) {
    CloseWindow();
  }
  Logger::instance().info("MAIN_WINDOW", "CabanaMainWindow destroyed");
}

void CabanaMainWindow::createUIComponents() {
  // Create UI components
  video_widget = std::make_unique<VideoWidget>();
  messages_widget = std::make_unique<MessagesWidget>();
  charts_widget = std::make_unique<ChartsWidget>();
  center_widget = std::make_unique<CenterWidget>();

  // Initialize component sizes and positions
  menu_bar_rect = {0, 0, 1200, 30};
  status_bar_rect = {0, 770, 1200, 30};
  messages_panel_rect = {0, 30, 400, 740};
  video_panel_rect = {400, 30, 400, 400};
  charts_panel_rect = {400, 430, 800, 340};
}

void CabanaMainWindow::run() {
  // Main application loop
  while (!WindowShouldClose() && running) {
    // Update the application
    update();

    // Render the UI
    render();
  }
}

void CabanaMainWindow::update() {
  // Update UI components
  handleInput();

  if (messages_widget) {
    messages_widget->update();
  }

  if (video_widget) {
    video_widget->update();
  }

  if (charts_widget) {
    charts_widget->update();
  }

  if (center_widget) {
    center_widget->update();
  }

  // Update status timeout
  if (status_timeout > 0) {
    double current_time = GetTime();
    if (current_time - last_time > status_timeout) {
      status_message.clear();
      status_timeout = 0.0f;
    }
  }
}

void CabanaMainWindow::render() {
  // Begin drawing
  BeginDrawing();
  ClearBackground((Color){245, 245, 245, 255}); // RAYWHITE equivalent

  // Render menu bar
  DrawRectangleRec(menu_bar_rect, (Color){211, 211, 211, 255}); // LIGHTGRAY in RGB
  DrawText("Cabana - CAN Bus Analyzer", 10, 10, 16, (Color){64, 64, 64, 255}); // DARKGRAY in RGB

  // Render status bar
  DrawRectangleRec(status_bar_rect, (Color){211, 211, 211, 255}); // LIGHTGRAY in RGB
  if (!status_message.empty()) {
    DrawText(status_message.c_str(), 10, 775, 12, (Color){64, 64, 64, 255}); // DARKGRAY in RGB
  }

  // Render UI components based on visibility settings
  if (show_messages_panel && messages_widget) {
    messages_widget->render(messages_panel_rect);
  }

  if (show_video_panel && video_widget) {
    video_widget->render(video_panel_rect);
  }

  if (show_charts_panel && charts_widget) {
    charts_widget->render(charts_panel_rect);
  }

  if (center_widget) {
    // Adjust center widget position based on other panels
    Rectangle center_rect = {800, 30, 400, 400};
    center_widget->render(center_rect);
  }

  // End drawing
  EndDrawing();
}

void CabanaMainWindow::handleInput() {
  // Handle keyboard input
  if (IsKeyPressed(KEY_F11)) {
    is_fullscreen = !is_fullscreen;
    ToggleFullscreen();
  }

  if (IsKeyPressed(KEY_ESCAPE)) {
    running = false;
  }
}

void CabanaMainWindow::showMessage(const std::string &msg, int timeout) {
  status_message = msg;
  status_timeout = timeout > 0 ? timeout / 1000.0f : 0.0f; // Convert to seconds
  last_time = GetTime();
}

void CabanaUIElement::handleInput() {
  // Base class implementation
}

void CabanaUIElement::setPosition(float x, float y) {
  bounds.x = x;
  bounds.y = y;
}

void CabanaUIElement::setSize(float width, float height) {
  bounds.width = width;
  bounds.height = height;
}

CabanaUIElement::CabanaUIElement(float x, float y, float width, float height)
  : bounds({x, y, width, height}), isVisible(true) {
}