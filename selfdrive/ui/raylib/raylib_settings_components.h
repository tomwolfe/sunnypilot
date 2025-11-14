#pragma once

#include "selfdrive/ui/raylib/raylib_ui_window.h"

// Settings window UI element
class SettingsWindowElement : public UIElement {
public:
  SettingsWindowElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;
  
  void setCurrentPanel(int index, const std::string &param = "");

private:
  int currentPanel = 0;
  std::string currentParam = "";
  
  void renderHeader() const;
  void renderSidebar() const;
  void renderPanelContent() const;
};

// Sidebar UI element
class SidebarElement : public UIElement {
public:
  SidebarElement(float x = 0, float y = 0, float width = 0, float height = 0);
  void update(const UIState &s) override;
  void render() const override;
  
  void offroadTransition(bool offroad);

private:
  bool isOffroad = true;
  float animationProgress = 0.0f;
  
  void renderItems() const;
};