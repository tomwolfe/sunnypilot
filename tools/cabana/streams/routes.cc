#include "tools/cabana/streams/routes.h"
#include "third_party/raylib/include/raylib.h"

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <sstream>

// HTTP request implementation for Raylib-based UI
// Implementation using standard HTTP libraries
namespace api {

class OneShotHttpRequest {
public:
  OneShotHttpRequest() = default;

  void send(const std::string &url, std::function<void(const std::string&, bool)> callback) {
    // Implementation would go here
    // For now, this is a placeholder
  }
};

} // namespace api

RoutesDialog::RoutesDialog(void* parent) {
  // Initialize Raylib-based route selection dialog
  visible_ = false;
  window_bounds_ = {200, 100, 600, 500};  // x, y, width, height
  device_list_bounds_ = {window_bounds_.x + 20, window_bounds_.y + 40, 200, 30};
  period_selector_bounds_ = {window_bounds_.x + 250, window_bounds_.y + 40, 150, 30};
  route_list_bounds_ = {window_bounds_.x + 20, window_bounds_.y + 90, 560, 380};
}

void RoutesDialog::show() {
  visible_ = true;
}

void RoutesDialog::hide() {
  visible_ = false;
}

void RoutesDialog::update() {
  // Handle input if visible
  if (visible_) {
    // Check for mouse clicks on UI elements
    Vector2 mouse_pos = GetMousePosition();

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      // Check if clicked in route list area
      if (CheckCollisionPointRec(mouse_pos, route_list_bounds_)) {
        // Calculate which route was clicked based on scroll and position
        float relative_y = mouse_pos.y - route_list_bounds_.y - 25; // Offset for header
        int row_height = 25;
        int clicked_row = static_cast<int>(relative_y / row_height);

        if (clicked_row >= 0 && clicked_row < static_cast<int>(routes_.size())) {
          selected_route_index_ = clicked_row;
          selected_route_ = routes_[clicked_row].route_key;

          // Call the callback if set
          if (onRouteSelected) {
            onRouteSelected(selected_route_);
          }
        }
      }
    }
  }
}

void RoutesDialog::render(const Rectangle& bounds) {
  if (!visible_) return;

  // Draw modal background
  DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade((Color){0, 0, 0, 255}, 0.5f));

  // Draw dialog window
  DrawRectangleRec(window_bounds_, (Color){245, 245, 245, 255});
  DrawRectangleLines(window_bounds_.x, window_bounds_.y, window_bounds_.width, window_bounds_.height, (Color){0, 0, 0, 255});

  // Draw title
  DrawText("Select Route", window_bounds_.x + 10, window_bounds_.y + 10, 20, (Color){0, 0, 0, 255});

  // Draw device selection
  DrawText("Device:", device_list_bounds_.x, device_list_bounds_.y - 20, 14, (Color){0, 0, 0, 255});
  DrawRectangleRec(device_list_bounds_, (Color){200, 200, 200, 255});
  DrawRectangleLines(device_list_bounds_.x, device_list_bounds_.y, device_list_bounds_.width, device_list_bounds_.height, (Color){0, 0, 0, 255});
  if (!devices_.empty() && selected_device_index_ >= 0) {
    DrawText(devices_[selected_device_index_].alias.c_str(),
             device_list_bounds_.x + 5, device_list_bounds_.y + 5, 12, (Color){0, 0, 0, 255});
  } else {
    DrawText("Select device...", device_list_bounds_.x + 5, device_list_bounds_.y + 5, 12, (Color){128, 128, 128, 255});
  }

  // Draw period selector
  DrawText("Period:", period_selector_bounds_.x, period_selector_bounds_.y - 20, 14, (Color){0, 0, 0, 255});
  DrawRectangleRec(period_selector_bounds_, (Color){200, 200, 200, 255});
  DrawRectangleLines(period_selector_bounds_.x, period_selector_bounds_.y, period_selector_bounds_.width, period_selector_bounds_.height, (Color){0, 0, 0, 255});
  const char* periods[] = {"Last Week", "Last Month", "Last 3 Months", "Last Year"};
  DrawText(periods[selected_period_index_], period_selector_bounds_.x + 5, period_selector_bounds_.y + 5, 12, (Color){0, 0, 0, 255});

  // Draw route list header
  DrawText("Route Key", route_list_bounds_.x + 5, route_list_bounds_.y, 12, (Color){0, 0, 0, 255});
  DrawText("Name", route_list_bounds_.x + 150, route_list_bounds_.y, 12, (Color){0, 0, 0, 255});
  DrawText("Date", route_list_bounds_.x + 300, route_list_bounds_.y, 12, (Color){0, 0, 0, 255});
  DrawText("Segments", route_list_bounds_.x + 450, route_list_bounds_.y, 12, (Color){0, 0, 0, 255});

  // Draw separator line
  DrawLine(route_list_bounds_.x, route_list_bounds_.y + 20,
           route_list_bounds_.x + route_list_bounds_.width, route_list_bounds_.y + 20, (Color){0, 0, 0, 255});

  // Draw route list
  float y_pos = route_list_bounds_.y + 25;
  for (size_t i = 0; i < routes_.size(); ++i) {
    // Highlight selected row
    if (static_cast<int>(i) == selected_route_index_) {
      DrawRectangle(route_list_bounds_.x, y_pos, route_list_bounds_.width, 25, (Color){180, 180, 255, 255});
    }

    // Draw row separator
    DrawRectangleLines(route_list_bounds_.x, y_pos, route_list_bounds_.width, 25, (Color){200, 200, 200, 255});

    // Draw route data
    DrawText(routes_[i].route_key.c_str(), route_list_bounds_.x + 5, y_pos + 5, 10, (Color){0, 0, 0, 255});
    DrawText(routes_[i].name.c_str(), route_list_bounds_.x + 150, y_pos + 5, 10, (Color){0, 0, 0, 255});
    DrawText(routes_[i].date.c_str(), route_list_bounds_.x + 300, y_pos + 5, 10, (Color){0, 0, 0, 255});
    DrawText(std::to_string(routes_[i].segment_count).c_str(), route_list_bounds_.x + 450, y_pos + 5, 10, (Color){0, 0, 0, 255});

    y_pos += 25;

    // If we're past the visible area, stop drawing
    if (y_pos >= route_list_bounds_.y + route_list_bounds_.height) {
      break;
    }
  }
}

void RoutesDialog::parseDeviceList(const std::string &json, bool success) {
  if (success) {
    // Parse device list from JSON
    // Implementation would go here
  }
}

void RoutesDialog::fetchRoutes() {
  // Fetch routes implementation
  // Implementation would go here
}

void RoutesDialog::parseRouteList(const std::string &json, bool success) {
  if (success) {
    // Parse route list from JSON
    // Implementation would go here
  }
}

std::string RoutesDialog::route() {
  return selected_route_;
}
