#include "selfdrive/ui/raylib/raylib_extra_components.h"
#include "selfdrive/ui/raylib/raylib_ui_state.h"
#include "selfdrive/ui/raylib/raylib_font_manager.h"
#include "selfdrive/ui/raylib/raylib_texture_manager.h"

// Implementation of DriverViewWindowElement
DriverViewWindowElement::DriverViewWindowElement(float x, float y, float width, float height) 
  : UIElement(x, y, width, height) {
}

void DriverViewWindowElement::update(const UIState &s) {
  // Update driver view logic
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    Vector2 mouse_pos = GetMousePosition();
    // Handle close button or other interactions
    if (mouse_pos.x > bounds.x + bounds.width - 80 && mouse_pos.y < bounds.y + 80) {
      // Clicked close button area
      if (doneCallback) {
        doneCallback();
      }
    }
  }
}

void DriverViewWindowElement::render() const {
  if (!isVisible) return;
  
  // Draw driver view background
  drawRect(bounds, UIColor(0, 0, 0, 255));  // Black background for camera view
  
  renderCameraView();
  renderOverlays();
}

void DriverViewWindowElement::renderCameraView() const {
  // Draw the camera feed (simulated)
  // This would show the actual driver camera feed in a real implementation
  Rectangle camera_rect = bounds;
  DrawRectangleRec(camera_rect, (Color){20, 20, 20, 255});  // Dark gray as placeholder
  
  // Draw some elements to represent camera data
  DrawCircle(bounds.x + bounds.width/2, bounds.y + bounds.height/2, 100, 
             (Color){80, 80, 80, 200});  // Center point
}

void DriverViewWindowElement::renderOverlays() const {
  // Draw UI overlays on top of camera view
  // Draw close button
  Rectangle close_btn = {bounds.x + bounds.width - 70, bounds.y + 20, 50, 50};
  DrawRectangleRec(close_btn, (Color){50, 50, 50, 200});
  drawText("X", close_btn.x + 15, close_btn.y + 10, 30, UIColor(255, 255, 255, 255));
  
  // Draw other overlays like grid lines, etc.
  for (int i = 1; i < 5; i++) {
    int x = bounds.x + i * (bounds.width / 5);
    DrawLine(x, bounds.y, x, bounds.y + bounds.height, (Color){100, 100, 100, 100});
  }
  
  for (int i = 1; i < 3; i++) {
    int y = bounds.y + i * (bounds.height / 3);
    DrawLine(bounds.x, y, bounds.x + bounds.width, y, (Color){100, 100, 100, 100});
  }
}

// Implementation of BodyWindowElement
BodyWindowElement::BodyWindowElement(float x, float y, float width, float height) 
  : UIElement(x, y, width, height) {
}

void BodyWindowElement::update(const UIState &s) {
  // Update body window logic
  // Body window is for non-car mode
}

void BodyWindowElement::render() const {
  if (!isVisible) return;
  
  // Draw body view background
  drawRect(bounds, UIColor(35, 35, 45, 255));
  
  renderBodyView();
  renderControls();
}

void BodyWindowElement::renderBodyView() const {
  // Draw body view content - this is for non-car mode
  std::string title = "Generic Robot Mode";
  drawText(title.c_str(), bounds.x + 100, bounds.y + 100, 48, UIColor(255, 255, 255, 255));
  
  std::string desc = "Operating in generic robot mode";
  drawText(desc.c_str(), bounds.x + 100, bounds.y + 180, 32, UIColor(200, 200, 200, 255));
}

void BodyWindowElement::renderControls() const {
  // Draw controls specific to body mode
  Rectangle control_area = {bounds.x + 50, bounds.y + 300, bounds.width - 100, 400};
  drawRect(control_area, UIColor(45, 45, 55, 200));
  
  drawText("Robot Controls", control_area.x + 20, control_area.y + 20, 36, 
           UIColor(255, 255, 255, 255));
}