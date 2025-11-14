#pragma once

#include <vector>
#include <string>
#include <memory>

#include "raylib.h"
#include "tools/cabana/streams/abstractstream.h"
#include "tools/cabana/chart/chartswidget.h"
#include "tools/cabana/dbc/dbcmanager.h"
#include "tools/cabana/detailwidget.h"
#include "tools/cabana/messageswidget.h"
#include "tools/cabana/videowidget.h"
#include "tools/cabana/tools/findsimilarbits.h"
#include "tools/cabana/raylib_centerwidget.h"

// Main window implementation using Raylib instead of QT
class CabanaMainWindow {
public:
  CabanaMainWindow();
  ~CabanaMainWindow();
  void run();
  void update();
  void render();
  void handleInput();
  void showMessage(const std::string &msg, int timeout = 0);

  // Public members for accessing UI components
  std::unique_ptr<VideoWidget> video_widget;
  std::unique_ptr<MessagesWidget> messages_widget;
  std::unique_ptr<ChartsWidget> charts_widget;
  std::unique_ptr<CenterWidget> center_widget;

private:
  void createUIComponents();
  void createMenu();
  void createStatusBar();
  void createDockWindows();

  bool running = true;
  std::string status_message;
  float status_timeout = 0.0f;
  double last_time = 0.0;

  // UI Component dimensions and positioning
  Rectangle menu_bar_rect;
  Rectangle messages_panel_rect;
  Rectangle video_panel_rect;
  Rectangle charts_panel_rect;
  Rectangle status_bar_rect;

  // UI State
  bool show_messages_panel = true;
  bool show_video_panel = true;
  bool show_charts_panel = true;
  bool is_fullscreen = false;
};

class CabanaUIElement {
public:
  CabanaUIElement(float x, float y, float width, float height);
  virtual ~CabanaUIElement() = default;

  virtual void update() = 0;
  virtual void render() = 0;
  virtual void handleInput();

  void setPosition(float x, float y);
  void setSize(float width, float height);
  Rectangle getBounds() const { return bounds; }
  void setVisible(bool visible) { isVisible = visible; }
  bool getVisible() const { return isVisible; }

protected:
  Rectangle bounds;
  bool isVisible;
};