#pragma once

#include "selfdrive/ui/raylib/raylib_ui_window.h"

// Body UI element (for body control view)
class BodyWindowElement : public UIElement {
public:
  BodyWindowElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;

private:
  float last_update_time = 0.0f;
  bool needs_update = true;

  void renderBodyControls() const;
  void renderVehicleStatus() const;
};

// Driver monitoring view UI element
class DriverViewWindowElement : public UIElement {
public:
  DriverViewWindowElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;

  // Callback for when driver view is done
  std::function<void()> doneCallback;

private:
  float last_update_time = 0.0f;
  bool needs_update = true;
  bool showDriverMonitoring = true;

  void renderDriverCamera() const;
  void renderDriverMonitoringData() const;
};