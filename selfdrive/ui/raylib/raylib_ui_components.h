#pragma once

#include "selfdrive/ui/raylib/raylib_ui_window.h"

// Offroad home screen UI element
class OffroadHomeElement : public UIElement {
public:
  OffroadHomeElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;

private:
  // UI elements that would be part of the offroad home screen
  float last_update_time = 0.0f;
  bool needs_update = true;
  
  void renderVisualizer() const;
  void renderStatusText() const;
  void renderButtons() const;
};

// Onroad UI element
class OnroadWindowElement : public UIElement {
public:
  OnroadWindowElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;

private:
  void renderDriverCamera() const;
  void renderUIElements() const;
  void renderAlerts() const;
};