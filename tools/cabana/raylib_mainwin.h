#pragma once

#include <memory>
#include <string>

// Forward declarations for UI components
class VideoWidget;
class MessagesWidget;
// Include Raylib for basic types like Rectangle
#include "raylib.h"

class CabanaUIElement {
public:
  CabanaUIElement(float x = 0, float y = 0, float width = 100, float height = 100);
  virtual ~CabanaUIElement() = default;
  
  virtual void render(const Rectangle &bounds) = 0;
  virtual void update() = 0;
  virtual void handleInput();
  
  void setPosition(float x, float y);
  void setSize(float width, float height);
  
  Rectangle bounds;
  bool isVisible;
};

class CabanaMainWindow {
public:
  CabanaMainWindow();
  ~CabanaMainWindow();
  
  void run();
  void update();
  void render();
  void handleInput();
  void createUIComponents();
  
  void showMessage(const std::string &msg, int timeout = 0);

private:
  // UI component pointers
  std::unique_ptr<VideoWidget> video_widget;
  std::unique_ptr<MessagesWidget> messages_widget;
  std::unique_ptr<class ChartsWidget> charts_widget;
  std::unique_ptr<class CenterWidget> center_widget;
  
  // UI element rectangles
  Rectangle menu_bar_rect;
  Rectangle status_bar_rect;
  Rectangle messages_panel_rect;
  Rectangle video_panel_rect;
  Rectangle charts_panel_rect;
  
  // State variables
  bool running = true;
  bool is_fullscreen = false;
  bool show_messages_panel = true;
  bool show_video_panel = true;
  bool show_charts_panel = true;
  
  // Status message
  std::string status_message;
  double status_timeout = 0.0;
  double last_time = 0.0;
};