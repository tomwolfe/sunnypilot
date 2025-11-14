#pragma once

#include "selfdrive/ui/raylib/raylib_ui_window.h"

// Driver view window UI element
class DriverViewWindowElement : public UIElement {
public:
  DriverViewWindowElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;
  
  // Signal for when view is done/closed
  std::function<void()> doneCallback;

private:
  void renderCameraView() const;
  void renderOverlays() const;
};

// Body window UI element (for not-car mode)
class BodyWindowElement : public UIElement {
public:
  BodyWindowElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;

private:
  bool isEnabled = false;
  
  void renderBodyView() const;
  void renderControls() const;
};