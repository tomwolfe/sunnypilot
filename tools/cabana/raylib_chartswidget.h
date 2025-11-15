#pragma once

#include <vector>
#include <string>
#include <memory>
#include <map>

#include "tools/cabana/dbc/dbcmanager.h"

// Define OPENPILOT_RAYLIB before including raylib to prevent enum conflicts
#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"

// Data structure for chart points
struct ChartPoint {
    double x;  // Time or index
    double y;  // Value
    MessageId msg_id;  // Associated message ID
    std::string signal_name;  // Associated signal name
};

// Chart configuration
struct ChartConfig {
    std::string title = "Chart";
    std::string x_label = "Time";
    std::string y_label = "Value";
    bool auto_scale = true;
    double min_y = 0.0;
    double max_y = 100.0;
    Color line_color = RAYLIB_BLUE;
    bool show_grid = true;
    bool show_legend = true;
};

// Raylib-based ChartsWidget
class ChartsWidget {
public:
    ChartsWidget(void* parent = nullptr);
    ~ChartsWidget();
    
    void update();
    void render(const Rectangle& bounds);
    void handleInput();
    
    // Chart management
    void addSignalToChart(const MessageId& msg_id, const std::string& signal_name);
    void removeSignalFromChart(const MessageId& msg_id, const std::string& signal_name);
    void clearChart();
    void setChartConfig(const ChartConfig& config);
    
    // Data management
    void addDataPoint(const ChartPoint& point);
    void addDataPoints(const std::vector<ChartPoint>& points);
    
    // UI state management
    void setIsDocked(bool docked) { is_docked_ = docked; }
    bool getIsDocked() const { return is_docked_; }
    
    // Signals equivalent
    std::function<void()> onToggleChartsDocking;
    std::function<void(double)> onShowTip;

private:
    void drawChartGrid(const Rectangle& chart_area);
    void drawChartData(const Rectangle& chart_area);
    void drawChartAxes(const Rectangle& chart_area);
    void drawLegend(const Rectangle& bounds);
    
    std::vector<ChartPoint> data_points_;
    std::map<std::string, std::vector<ChartPoint>> signal_data_;  // Group data by signal
    ChartConfig chart_config_;
    
    Rectangle bounds_;
    bool is_visible_ = true;
    bool is_docked_ = true;
    
    // UI interaction state
    Vector2 last_mouse_pos_ = {0, 0};
    bool is_dragging_ = false;
    bool is_zooming_ = false;
    
    // Chart view state
    double x_min_ = 0.0;
    double x_max_ = 100.0;
    double y_min_ = 0.0;
    double y_max_ = 100.0;
    double x_offset_ = 0.0;
    double y_offset_ = 0.0;
    double zoom_factor_ = 1.0;
    
    // Colors for different signals
    std::vector<Color> signal_colors_ = {
        RAYLIB_BLUE, RAYLIB_RED, RAYLIB_GREEN, RAYLIB_YELLOW, RAYLIB_PURPLE, RAYLIB_ORANGE, RAYLIB_PINK, RAYLIB_DARKBLUE, RAYLIB_DARKGREEN, RAYLIB_MAROON
    };
    int next_color_index_ = 0;
};