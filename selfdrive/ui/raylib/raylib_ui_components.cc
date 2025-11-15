#include "selfdrive/ui/raylib/raylib_ui_components.h"
#include "selfdrive/ui/raylib/raylib_ui_state_full.h"
#include "selfdrive/ui/raylib/raylib_font_manager.h"
#include "selfdrive/ui/raylib/raylib_texture_manager.h"

// Implementation of OffroadHomeElement
OffroadHomeElement::OffroadHomeElement(float x, float y, float width, float height) 
  : UIElement(x, y, width, height) {
}

void OffroadHomeElement::update(const UIState &s) {
  // Update logic for offroad home screen
  last_update_time = GetTime();
  needs_update = true;
  
  // Handle any interactions specific to the offroad home
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    Vector2 mouse_pos = GetMousePosition();
    // Handle button clicks, etc.
  }
}

void OffroadHomeElement::render() const {
  if (!isVisible) return;
  
  // Render offroad home screen
  // This would include the visualizer, status info, and navigation buttons
  
  renderVisualizer();
  renderStatusText();
  renderButtons();
}

void OffroadHomeElement::renderVisualizer() const {
  // Draw the visualizer rectangle
  Rectangle viz_rect = {bounds.x + 50, bounds.y + 50, bounds.width - 100, 400};
  drawRect(viz_rect, UIColor(30, 30, 40, 200));  // Dark background for visualizer
  
  // Draw some basic visual elements to represent the car/model status
  DrawRectangle(viz_rect.x + 50, viz_rect.y + 300, viz_rect.width - 100, 20, 
                (Color){70, 91, 234, 255});  // Blue bar representing car
}

void OffroadHomeElement::renderStatusText() const {
  // Draw status text based on UI state
  std::string status_text = "Sunnypilot Offroad";
  drawText(status_text.c_str(), bounds.x + 100, bounds.y + 500, 48, 
           UIColor(255, 255, 255, 255));
           
  // Draw additional status information
  std::string conn_status = "Connected"; // This would come from actual state
  drawText(conn_status.c_str(), bounds.x + 100, bounds.y + 560, 36, 
           UIColor(200, 200, 200, 255));
}

void OffroadHomeElement::renderButtons() const {
  // Draw settings button
  Rectangle settings_btn = {bounds.x + 100, bounds.y + 650, 200, 80};
  DrawRectangleRec(settings_btn, (Color){60, 60, 70, 255});
  DrawRectangleLinesEx(settings_btn, 2, (Color){100, 100, 110, 255});
  drawText("Settings", settings_btn.x + 30, settings_btn.y + 25, 36, 
           UIColor(255, 255, 255, 255));
  
  // Draw other buttons as needed
  Rectangle drive_btn = {bounds.x + 350, bounds.y + 650, 200, 80};
  DrawRectangleRec(drive_btn, (Color){60, 60, 70, 255});
  DrawRectangleLinesEx(drive_btn, 2, (Color){100, 100, 110, 255});
  drawText("Drive", drive_btn.x + 50, drive_btn.y + 25, 36, 
           UIColor(255, 255, 255, 255));
}

// Implementation of OnroadWindowElement
OnroadWindowElement::OnroadWindowElement(float x, float y, float width, float height)
  : UIElement(x, y, width, height) {
}

void OnroadWindowElement::update(const UIState &s) {
  // Update logic for onroad screen
  last_update_time = GetTime();
  needs_update = true;

  // Store the current status and scene for rendering
  currentStatus = s.status;
  currentScene = s.scene;

  // Handle any onroad-specific interactions
}

void OnroadWindowElement::render() const {
  if (!isVisible) return;

  // Render onroad screen elements
  renderDriverCamera();
  renderUIElements();
  renderAlerts();
}

void OnroadWindowElement::renderDriverCamera() const {
  // Draw camera view area
  Rectangle camera_rect = {bounds.x, bounds.y, bounds.width, bounds.height};
  // In a real implementation, this would draw the actual camera feed
  DrawRectangleRec(camera_rect, (Color){0, 0, 0, 255});  // Black background for camera
  
  // Draw some mock camera elements for visualization
  DrawRectangle(bounds.x + 50, bounds.y + 50, bounds.width - 100, 20, 
                (Color){255, 255, 0, 200});  // Yellow line representing lane
  DrawRectangle(bounds.x + 50, bounds.y + bounds.height - 100, bounds.width - 100, 20, 
                (Color){255, 255, 0, 200});  // Bottom lane line
}

void OnroadWindowElement::renderUIElements() const {
  // Draw speed, status, and other driving information
  // This would be positioned appropriately on the screen

  // Draw status bar at top
  Rectangle status_bar = {bounds.x, bounds.y, bounds.width, 80};
  drawRect(status_bar, bg_colors[currentStatus]);  // Use status-appropriate color

  // Draw speed or other info
  std::string speed_text = "0 MPH";
  if (currentScene.started && currentScene.started_frame > 0) {
    // In a real implementation, get actual speed from state
    speed_text = "45 MPH";
  }
  drawText(speed_text.c_str(), bounds.x + 50, bounds.y + 20, 48,
           UIColor(255, 255, 255, 255));
}

void OnroadWindowElement::renderAlerts() const {
  // Draw any alerts if present
  // This would check the UI state for any active alerts to display
}