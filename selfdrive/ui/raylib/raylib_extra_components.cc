#include "selfdrive/ui/raylib/raylib_extra_components.h"
#include "selfdrive/ui/raylib/raylib_ui_state_full.h"
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

  // Render driver monitoring screen
  renderDriverCamera();
  renderDriverMonitoringData();
}

void DriverViewWindowElement::renderDriverCamera() const {
  // Draw full-screen camera view for driver monitoring
  Rectangle camera_rect = {bounds.x, bounds.y, bounds.width, bounds.height};
  // In a real implementation, this would draw the actual driver camera feed
  DrawRectangleRec(camera_rect, (Color){20, 20, 30, 255});  // Dark background

  // Draw some mock driver camera elements for visualization
  DrawCircle(bounds.width/2, bounds.height/2, 150, (Color){100, 100, 150, 200});  // Head position
  DrawCircle(bounds.width/2 - 60, bounds.height/2 - 30, 20, (Color){255, 255, 255, 255});  // Left eye
  DrawCircle(bounds.width/2 + 60, bounds.height/2 - 30, 20, (Color){255, 255, 255, 255});  // Right eye
  DrawRectangle(bounds.width/2 - 40, bounds.height/2 + 30, 80, 20, (Color){200, 200, 200, 200});  // Mouth

  // Draw exit button
  Rectangle exit_btn = {bounds.x + 20, bounds.y + 20, 60, 60};
  DrawRectangleRec(exit_btn, (Color){50, 50, 60, 200});
  drawText("X", bounds.x + 35, bounds.y + 30, 36, UIColor(255, 255, 255, 255));
}

void DriverViewWindowElement::renderDriverMonitoringData() const {
  // Draw driver monitoring information overlay
  // This would show drowsiness, attention status, etc.
  std::string monitoring_status = "Driver Status: Attentive";
  Rectangle status_rect = {bounds.x + 20, bounds.y + bounds.height - 80, bounds.width - 40, 60};
  drawRect(status_rect, UIColor(30, 30, 40, 200));
  drawText(monitoring_status.c_str(), bounds.x + 40, bounds.y + bounds.height - 60, 30,
           UIColor(200, 255, 200, 255));
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

  // Render body control screen elements
  renderBodyControls();
  renderVehicleStatus();
}

void BodyWindowElement::renderBodyControls() const {
  // Draw body control interface - for controlling vehicle systems
  Rectangle control_rect = {bounds.x + 50, bounds.y + 50, bounds.width - 100, bounds.height - 100};
  drawRect(control_rect, UIColor(40, 40, 50, 200));

  // Draw title
  drawText("Body Controls", bounds.x + 100, bounds.y + 80, 48, UIColor(255, 255, 255, 255));

  // Draw control buttons for various body functions
  // Example: door locks, windows, trunk, etc.
  std::vector<std::string> controls = {
    "Door Locks", "Windows", "Trunk", "Horn", "Lights"
  };

  for (size_t i = 0; i < controls.size(); ++i) {
    int btn_y = bounds.y + 180 + (i * 80);
    Rectangle btn_rect = {bounds.x + 100, btn_y, bounds.width - 200, 60};

    // Draw button background
    DrawRectangleRec(btn_rect, (Color){60, 60, 70, 255});
    DrawRectangleLinesEx(btn_rect, 2, (Color){100, 100, 110, 255});

    // Draw button text
    drawText(controls[i].c_str(), btn_rect.x + 20, btn_rect.y + 15, 30, UIColor(255, 255, 255, 255));
  }
}

void BodyWindowElement::renderVehicleStatus() const {
  // Draw vehicle status information
  drawText("Vehicle Status", bounds.x + 100, bounds.y + 650, 36, UIColor(200, 200, 200, 255));

  // Draw additional vehicle information
  // This would connect to actual vehicle data
  std::string status_info = "Connected";
  drawText(status_info.c_str(), bounds.x + 100, bounds.y + 700, 30, UIColor(200, 200, 200, 255));
}