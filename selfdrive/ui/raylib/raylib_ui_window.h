#pragma once

#include <raylib.h>
#include <functional>
#include <vector>
#include <memory>

#include "selfdrive/ui/raylib/raylib_ui_state.h"

// Forward declare UI components
class OffroadHomeElement;
class OnroadWindowElement;
class SettingsWindowElement;
class SidebarElement;
class DriverViewWindowElement;
class BodyWindowElement;

// Basic UI element class that all UI components will inherit from
class UIElement {
public:
  UIElement(float x = 0, float y = 0, float width = 0, float height = 0);
  virtual ~UIElement() = default;

  virtual void update(const UIState &s) = 0;
  virtual void render() const = 0;

  void setPosition(float x, float y);
  void setSize(float width, float height);
  Rectangle getBounds() const { return bounds; }

  void setVisible(bool visible) { isVisible = visible; }
  bool getVisible() const { return isVisible; }

protected:
  Rectangle bounds;
  bool isVisible = true;
};

// The main UI window that manages different UI states (offroad/onroad)
class RaylibMainWindow {
public:
  RaylibMainWindow();
  ~RaylibMainWindow();

  void update(const UIState &s);
  void render(const UIState &s);

  void openSettings(int index = 0, const std::string &param = "");
  void closeSettings();
  void showDriverView(bool show);

private:
  // UI elements
  std::unique_ptr<SidebarElement> sidebar;
  std::unique_ptr<OffroadHomeElement> offroadHome;
  std::unique_ptr<OnroadWindowElement> onroadWindow;
  std::unique_ptr<SettingsWindowElement> settingsWindow;
  std::unique_ptr<BodyWindowElement> bodyWindow;
  std::unique_ptr<DriverViewWindowElement> driverViewWindow;

  // Current active UI state
  enum class UIStateType {
    OFFROAD_HOME,
    ONROAD,
    BODY,
    DRIVER_VIEW,
    SETTINGS
  };

  UIStateType currentUIState = UIStateType::OFFROAD_HOME;

  // Internal methods
  void updateActiveUI(const UIState &s);
  void renderActiveUI() const;
};

// Basic UI rendering functions
void drawBackground(const UIColor &color);
void drawRect(const Rectangle &rect, const UIColor &color);
void drawText(const char* text, float x, float y, int fontSize, const UIColor &color);
void drawImage(const char* imagePath, float x, float y, float width, float height);

// Input handling
bool isMouseClicked();
Vector2 getMousePosition();