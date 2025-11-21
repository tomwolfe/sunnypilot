#pragma once

#include <string>
#include <vector>
#include <functional>

#include "third_party/raylib/include/raylib.h"
#include "tools/cabana/utils/api.h"

// Structure to hold route information
struct RouteInfo {
    std::string route_key;
    std::string name;
    std::string date;
    int segment_count;
};

// Structure to hold device information
struct DeviceInfo {
    std::string dongle_id;
    std::string alias;
    std::string last_acked_timestamp;
};

// Raylib-based RoutesDialog
class RoutesDialog {
public:
    RoutesDialog(void* parent = nullptr);
    std::string route();

    void show();
    void hide();
    bool isVisible() const { return visible_; }
    void update();
    void render(const Rectangle& bounds);

    // Callback for when a route is selected
    std::function<void(const std::string&)> onRouteSelected;

private:
    void parseDeviceList(const std::string &json, bool success);
    void parseRouteList(const std::string &json, bool success);
    void fetchRoutes();

    bool visible_ = false;
    std::vector<DeviceInfo> devices_;
    std::vector<RouteInfo> routes_;
    std::string selected_route_;

    // UI state
    int selected_device_index_ = -1;
    int selected_period_index_ = 0; // 0: week, 1: month, etc.
    int selected_route_index_ = -1;

    // Display bounds
    Rectangle window_bounds_;
    Rectangle device_list_bounds_;
    Rectangle period_selector_bounds_;
    Rectangle route_list_bounds_;
};
