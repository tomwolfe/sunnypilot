#include "raylib_chartswidget.h"
#include <cmath>  // for sin function

ChartsWidget::ChartsWidget(void* parent) {
    // Initialize with default chart configuration
    chart_config_.title = "CAN Signal Charts";
    chart_config_.x_label = "Time (s)";
    chart_config_.y_label = "Value";
    chart_config_.auto_scale = true;
    chart_config_.line_color = RAYLIB_BLUE;
    chart_config_.show_grid = true;
    chart_config_.show_legend = true;
    
    // Add some sample data for demonstration
    for (int i = 0; i < 100; ++i) {
        ChartPoint point;
        point.x = i * 0.1;  // 0.1 second intervals
        point.y = std::sin(i * 0.1) * 50 + 50;  // Sine wave
        point.msg_id = {0, 0x100};  // Example message ID
        point.signal_name = "STEERING_ANGLE";
        data_points_.push_back(point);
        
        // Add to signal-specific data
        signal_data_["STEERING_ANGLE"].push_back(point);
    }
}

ChartsWidget::~ChartsWidget() = default;

void ChartsWidget::update() {
    // Update chart state
    if (chart_config_.auto_scale) {
        // Auto-scale y-axis based on data
        if (!data_points_.empty()) {
            y_min_ = data_points_[0].y;
            y_max_ = data_points_[0].y;
            
            for (const auto& point : data_points_) {
                if (point.y < y_min_) y_min_ = point.y;
                if (point.y > y_max_) y_max_ = point.y;
            }
            
            // Add some padding
            double range = y_max_ - y_min_;
            if (range > 0) {
                y_min_ -= range * 0.05;
                y_max_ += range * 0.05;
            } else {
                y_min_ -= 1.0;
                y_max_ += 1.0;
            }
        }
    }
}

void ChartsWidget::render(const Rectangle& bounds) {
    if (!is_visible_) return;
    
    bounds_ = bounds;
    
    // Draw panel background
    DrawRectangleRec(bounds, Color{250, 250, 250, 255}); // Light background
    DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, RAYLIB_LIGHTGRAY);

    // Draw title
    DrawText(chart_config_.title.c_str(), bounds.x + 10, bounds.y + 5, 16, RAYLIB_DARKGRAY);

    // Calculate chart area (leaving space for labels and legend)
    Rectangle chart_area = {bounds.x + 60, bounds.y + 30, bounds.width - 80, bounds.height - 70};
    if (chart_config_.show_legend) {
        chart_area.height -= 40;  // Make room for legend
    }

    // Draw chart background
    DrawRectangleRec(chart_area, RAYLIB_RAYWHITE);
    DrawRectangleLines(chart_area.x, chart_area.y, chart_area.width, chart_area.height, RAYLIB_GRAY);

    // Draw grid if enabled
    if (chart_config_.show_grid) {
        drawChartGrid(chart_area);
    }

    // Draw axes
    drawChartAxes(chart_area);

    // Draw chart data
    drawChartData(chart_area);

    // Draw legend if enabled
    if (chart_config_.show_legend) {
        drawLegend(bounds);
    }

    // Draw axis labels
    DrawText(chart_config_.x_label.c_str(),
             bounds.x + bounds.width / 2 - MeasureText(chart_config_.x_label.c_str(), 10) / 2,
             bounds.y + bounds.height - 20, 10, RAYLIB_DARKGRAY);
             
    // Rotate and draw Y label
    // For simplicity, we'll just draw it normally; in a full implementation we'd rotate it
    DrawText(chart_config_.y_label.c_str(), bounds.x + 5, bounds.y + bounds.height / 2, 10, RAYLIB_DARKGRAY);
}

void ChartsWidget::drawChartGrid(const Rectangle& chart_area) {
    // Draw horizontal grid lines
    int grid_lines = 10;
    for (int i = 1; i < grid_lines; ++i) {
        float y = chart_area.y + (chart_area.height * i) / grid_lines;
        DrawLine(chart_area.x, y, chart_area.x + chart_area.width, y, RAYLIB_LIGHTGRAY);
    }

    // Draw vertical grid lines
    for (int i = 1; i < grid_lines; ++i) {
        float x = chart_area.x + (chart_area.width * i) / grid_lines;
        DrawLine(x, chart_area.y, x, chart_area.y + chart_area.height, RAYLIB_LIGHTGRAY);
    }
}

void ChartsWidget::drawChartAxes(const Rectangle& chart_area) {
    // Draw X and Y axes
    DrawLine(chart_area.x, chart_area.y + chart_area.height,
             chart_area.x + chart_area.width, chart_area.y + chart_area.height, RAYLIB_BLACK);  // X-axis
    DrawLine(chart_area.x, chart_area.y,
             chart_area.x, chart_area.y + chart_area.height, RAYLIB_BLACK);  // Y-axis

    // Draw axis ticks and labels
    // X-axis labels
    for (int i = 0; i <= 5; ++i) {
        float x = chart_area.x + (chart_area.width * i) / 5;
        DrawLine(x, chart_area.y + chart_area.height, x, chart_area.y + chart_area.height + 5, RAYLIB_BLACK);

        double value = x_min_ + (x_max_ - x_min_) * i / 5;
        char label[32];
        snprintf(label, sizeof(label), "%.1f", value);
        DrawText(label, x - 10, chart_area.y + chart_area.height + 8, 8, RAYLIB_DARKGRAY);
    }

    // Y-axis labels
    for (int i = 0; i <= 5; ++i) {
        float y = chart_area.y + chart_area.height - (chart_area.height * i) / 5;
        DrawLine(chart_area.x - 5, y, chart_area.x, y, RAYLIB_BLACK);

        double value = y_min_ + (y_max_ - y_min_) * i / 5;
        char label[32];
        snprintf(label, sizeof(label), "%.1f", value);
        int label_width = MeasureText(label, 8);
        DrawText(label, chart_area.x - label_width - 8, y - 5, 8, RAYLIB_DARKGRAY);
    }
}

void ChartsWidget::drawChartData(const Rectangle& chart_area) {
    if (data_points_.empty()) return;
    
    // Determine the range of data to display based on current view
    double data_x_min = data_points_.front().x;
    double data_x_max = data_points_.back().x;
    
    // Map data values to screen coordinates
    auto mapX = [&](double value) -> float {
        return chart_area.x + ((value - data_x_min) / (data_x_max - data_x_min)) * chart_area.width;
    };
    
    auto mapY = [&](double value) -> float {
        return chart_area.y + chart_area.height - ((value - y_min_) / (y_max_ - y_min_)) * chart_area.height;
    };
    
    // Draw each signal's data
    int signal_index = 0;
    for (const auto& [signal_name, points] : signal_data_) {
        Color color = signal_colors_[signal_index % signal_colors_.size()];
        
        // Draw the line connecting points
        if (points.size() > 1) {
            for (size_t i = 1; i < points.size(); ++i) {
                Vector2 start = {mapX(points[i-1].x), mapY(points[i-1].y)};
                Vector2 end = {mapX(points[i].x), mapY(points[i].y)};
                
                // Only draw if both points are within chart area
                if (start.x >= chart_area.x && start.x <= chart_area.x + chart_area.width &&
                    end.x >= chart_area.x && end.x <= chart_area.x + chart_area.width) {
                    DrawLineV(start, end, color);
                }
            }
        }
        
        // Draw data points
        for (const auto& point : points) {
            Vector2 pos = {mapX(point.x), mapY(point.y)};
            if (pos.x >= chart_area.x && pos.x <= chart_area.x + chart_area.width &&
                pos.y >= chart_area.y && pos.y <= chart_area.y + chart_area.height) {
                DrawCircleV(pos, 2, color);
            }
        }
        
        signal_index++;
    }
}

void ChartsWidget::drawLegend(const Rectangle& bounds) {
    // Draw legend at the bottom of the chart
    Rectangle legend_area = {bounds.x + 10, bounds.y + bounds.height - 35, bounds.width - 20, 30};
    
    // Background for legend
    DrawRectangleRec(legend_area, Color{240, 240, 240, 255});
    DrawRectangleLines(legend_area.x, legend_area.y, legend_area.width, legend_area.height, RAYLIB_LIGHTGRAY);

    int signal_index = 0;
    int current_x = legend_area.x + 5;

    for (const auto& [signal_name, points] : signal_data_) {
        if (signal_index >= 5) break;  // Limit number of items in legend for space

        Color color = signal_colors_[signal_index % signal_colors_.size()];

        // Draw color indicator
        DrawRectangle(current_x, legend_area.y + 10, 10, 10, color);

        // Draw signal name
        DrawText(signal_name.c_str(), current_x + 15, legend_area.y + 8, 10, RAYLIB_DARKGRAY);
        
        current_x += 15 + MeasureText(signal_name.c_str(), 10) + 10;
        signal_index++;
    }
}

void ChartsWidget::handleInput() {
    Vector2 mouse_pos = GetMousePosition();
    
    // Handle zooming with mouse wheel
    float wheel_move = GetMouseWheelMove();
    if (wheel_move != 0 && CheckCollisionPointRec(mouse_pos, bounds_)) {
        is_zooming_ = true;
        float zoom_factor = wheel_move > 0 ? 1.1f : 0.9f;
        zoom_factor_ *= zoom_factor;
        
        // Limit zoom
        if (zoom_factor_ < 0.1f) zoom_factor_ = 0.1f;
        if (zoom_factor_ > 10.0f) zoom_factor_ = 10.0f;
    } else {
        is_zooming_ = false;
    }
    
    // Handle panning with mouse drag
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointRec(mouse_pos, bounds_)) {
            if (!is_dragging_) {
                last_mouse_pos_ = mouse_pos;
                is_dragging_ = true;
            } else {
                Vector2 delta = {mouse_pos.x - last_mouse_pos_.x, mouse_pos.y - last_mouse_pos_.y};
                
                // Adjust offsets based on panning
                x_offset_ += delta.x;
                y_offset_ += delta.y;
                
                last_mouse_pos_ = mouse_pos;
            }
        }
    } else {
        is_dragging_ = false;
    }
    
    // Handle tooltip showing on hover
    if (CheckCollisionPointRec(mouse_pos, bounds_)) {
        // Calculate the time at mouse position
        double data_x_min = !data_points_.empty() ? data_points_.front().x : 0;
        double data_x_max = !data_points_.empty() ? data_points_.back().x : 100;
        
        float chart_x = mouse_pos.x - (bounds_.x + 60); // Account for left padding
        if (chart_x >= 0 && chart_x <= bounds_.width - 80) { // Account for right padding
            double time_at_mouse = data_x_min + (data_x_max - data_x_min) * (chart_x / (bounds_.width - 80));
            
            if (onShowTip) {
                onShowTip(time_at_mouse);
            }
        }
    }
}

void ChartsWidget::addSignalToChart(const MessageId& msg_id, const std::string& signal_name) {
    // In a full implementation, this would start tracking the signal
    // For now, we'll just add it to a list of tracked signals
    if (signal_data_.find(signal_name) == signal_data_.end()) {
        signal_data_[signal_name] = std::vector<ChartPoint>();
    }
}

void ChartsWidget::removeSignalFromChart(const MessageId& msg_id, const std::string& signal_name) {
    signal_data_.erase(signal_name);
}

void ChartsWidget::clearChart() {
    data_points_.clear();
    signal_data_.clear();
}

void ChartsWidget::setChartConfig(const ChartConfig& config) {
    chart_config_ = config;
}

void ChartsWidget::addDataPoint(const ChartPoint& point) {
    data_points_.push_back(point);
    
    // Add to signal-specific data
    signal_data_[point.signal_name].push_back(point);
}

void ChartsWidget::addDataPoints(const std::vector<ChartPoint>& points) {
    for (const auto& point : points) {
        data_points_.push_back(point);
        signal_data_[point.signal_name].push_back(point);
    }
}