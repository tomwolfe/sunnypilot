#include "selfdrive/ui/raylib/raylib_settings_components.h"
#include "selfdrive/ui/raylib/raylib_ui_state.h"
#include "selfdrive/ui/raylib/raylib_font_manager.h"
#include "selfdrive/ui/raylib/raylib_texture_manager.h"

// Implementation of SettingsWindowElement
SettingsWindowElement::SettingsWindowElement(float x, float y, float width, float height) 
  : UIElement(x, y, width, height) {
  // Initialize with default values
}

void SettingsWindowElement::update(const UIState &s) {
  // Update settings window logic
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    Vector2 mouse_pos = GetMousePosition();
    // Handle settings panel interactions
  }
}

void SettingsWindowElement::render() const {
  if (!isVisible) return;
  
  // Draw settings window background
  drawRect(bounds, UIColor(25, 25, 35, 255));
  
  renderHeader();
  renderSidebar();
  renderPanelContent();
}

void SettingsWindowElement::setCurrentPanel(int index, const std::string &param) {
  currentPanel = index;
  currentParam = param;
}

void SettingsWindowElement::renderHeader() const {
  // Draw header with "Settings" title
  Rectangle header_rect = {bounds.x, bounds.y, bounds.width, 100};
  drawRect(header_rect, UIColor(40, 40, 50, 255));
  
  drawText("Settings", bounds.x + 50, bounds.y + 30, 48, UIColor(255, 255, 255, 255));
}

void SettingsWindowElement::renderSidebar() const {
  // Draw sidebar with navigation items
  Rectangle sidebar_rect = {bounds.x, bounds.y + 100, 250, bounds.height - 100};
  drawRect(sidebar_rect, UIColor(35, 35, 45, 255));
  
  // Draw some example menu items
  std::vector<std::string> menu_items = {"Device", "Network", "Toggles", "Software", "SSH Keys", "UI", "Sunnypilot"};
  
  for (size_t i = 0; i < menu_items.size(); ++i) {
    int item_y = bounds.y + 120 + (i * 60);
    Rectangle item_rect = {bounds.x, item_y, 250, 60};
    
    // Highlight selected item
    if (static_cast<int>(i) == currentPanel) {
      drawRect(item_rect, UIColor(60, 60, 80, 255));
    }
    
    drawText(menu_items[i].c_str(), bounds.x + 20, item_y + 15, 30, UIColor(255, 255, 255, 255));
  }
}

void SettingsWindowElement::renderPanelContent() const {
  // Draw the main content area for the selected panel
  Rectangle content_rect = {bounds.x + 250, bounds.y + 100, bounds.width - 250, bounds.height - 100};
  drawRect(content_rect, UIColor(30, 30, 40, 255));
  
  // Draw panel-specific content based on currentPanel
  std::string panel_title = "Settings Panel";
  switch (currentPanel) {
    case 0: panel_title = "Device"; break;
    case 1: panel_title = "Network"; break;
    case 2: panel_title = "Toggles"; break;
    case 3: panel_title = "Software"; break;
    case 4: panel_title = "SSH Keys"; break;
    case 5: panel_title = "UI"; break;
    case 6: panel_title = "Sunnypilot"; break;
    default: panel_title = "Settings Panel"; break;
  }
  
  drawText(panel_title.c_str(), content_rect.x + 30, content_rect.y + 30, 36, UIColor(255, 255, 255, 255));
}

// Implementation of SidebarElement
SidebarElement::SidebarElement(float x, float y, float width, float height) 
  : UIElement(x, y, width, height) {
  // Initialize with the typical sidebar dimensions
  bounds.width = 200; // Standard sidebar width
}

void SidebarElement::update(const UIState &s) {
  // Update sidebar logic
  isOffroad = !s.scene.started;
  
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    Vector2 mouse_pos = GetMousePosition();
    // Handle sidebar item clicks
    if (mouse_pos.x < bounds.width) {  // Clicked within sidebar area
      // Could emit signal to open settings or toggle sidebar
    }
  }
}

void SidebarElement::render() const {
  if (!isVisible) return;
  
  // Draw sidebar background
  drawRect(bounds, UIColor(30, 30, 40, 255));
  
  renderItems();
}

void SidebarElement::offroadTransition(bool offroad) {
  isOffroad = offroad;
  // Update visibility or animation based on transition
  if (!offroad) {
    // Onroad - maybe reduce visibility or hide automatically
    animationProgress = 0.0f;
  } else {
    // Offroad - sidebar fully visible
    animationProgress = 1.0f;
  }
}

void SidebarElement::renderItems() const {
  // Draw sidebar items like settings icon, etc.
  std::vector<std::string> items = {"Settings", "Map", "Volume", "Bluetooth"};
  
  for (size_t i = 0; i < items.size(); ++i) {
    int item_y = bounds.y + 150 + (i * 80);
    drawText(items[i].c_str(), bounds.x + 20, item_y, 32, UIColor(220, 220, 220, 255));
  }
  
  // Draw settings gear icon or similar
  DrawRectangle(bounds.x + 20, bounds.y + 50, 40, 40, (Color){70, 91, 234, 255});
}