#include "raylib_chartswidget.h"
#include <algorithm>
#include <cmath>

ChartsWidget::ChartsWidget(void* parent) {
  // Initialize with some sample data
  clear();
}

ChartsWidget::~ChartsWidget() {
  if (connected_to_stream_) {
    if (stream_ && event_subscription_id_ != 0) {
      // Note: We don't have a direct way to unsubscribe from all events in the current setup
      // This would need to be handled by the event system design
    }
  }
}

void ChartsWidget::update() {
  handleMouseInteraction();
  updateDataRange();
}

void ChartsWidget::render(const Rectangle& bounds) {
  if (!is_visible) return;

  bounds_ = bounds;

  // Draw panel background
  DrawRectangleRec(bounds, Color{240, 240, 240, 255}); // Light gray
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, (Color){211, 211, 211, 255}); // LIGHTGRAY in RGB

  // Draw title
  DrawText(title.c_str(), bounds.x + 10, bounds.y + 5, 14, (Color){64, 64, 64, 255}); // DARKGRAY in RGB

  // Draw chart area
  Rectangle chart_bounds = {
    bounds.x + 60,  // Leave space for Y-axis labels
    bounds.y + 20,  // Below title
    bounds.width - 60 - 20, // Leave space for Y-axis and scrollbar
    bounds.height - 40      // Leave space for X-axis and bottom
  };
  
  // Draw chart background
  DrawRectangleRec(chart_bounds, WHITE);
  DrawRectangleLines(chart_bounds.x, chart_bounds.y, chart_bounds.width, chart_bounds.height, (Color){200, 200, 200, 255});

  // Draw grid and axes
  drawGrid(chart_bounds);
  drawAxes(chart_bounds);

  // Draw the signal data
  if (!data_points_.empty()) {
    drawSignal(chart_bounds);
  }
}

void ChartsWidget::drawGrid(const Rectangle& bounds) {
  // Draw grid lines
  const int num_vertical_lines = 10;
  const int num_horizontal_lines = 5;
  
  // Vertical grid lines (time)
  for (int i = 1; i < num_vertical_lines; i++) {
    float x = bounds.x + (bounds.width * i / num_vertical_lines);
    DrawLine(x, bounds.y, x, bounds.y + bounds.height, (Color){230, 230, 230, 255});
  }
  
  // Horizontal grid lines (value)
  for (int i = 1; i < num_horizontal_lines; i++) {
    float y = bounds.y + (bounds.height * i / num_horizontal_lines);
    DrawLine(bounds.x, y, bounds.x + bounds.width, y, (Color){230, 230, 230, 255});
  }
}

void ChartsWidget::drawAxes(const Rectangle& bounds) {
  // Draw X and Y axes
  DrawLine(bounds.x, bounds.y + bounds.height, bounds.x + bounds.width, bounds.y + bounds.height, BLACK); // X-axis
  DrawLine(bounds.x, bounds.y, bounds.x, bounds.y + bounds.height, BLACK); // Y-axis
  
  // Draw axis labels if we have data
  if (!data_points_.empty()) {
    char label_buffer[50];
    
    // X-axis labels (time)
    float time_step = (max_time_ - min_time_) / 5;
    for (int i = 0; i <= 5; i++) {
      float time_val = min_time_ + time_step * i;
      snprintf(label_buffer, sizeof(label_buffer), "%.1fs", time_val);
      float x_pos = bounds.x + (bounds.width * i / 5);
      DrawText(label_buffer, x_pos - 15, bounds.y + bounds.height + 5, 8, BLACK);
    }
    
    // Y-axis labels (value)
    float value_step = (max_value_ - min_value_) / 5;
    for (int i = 0; i <= 5; i++) {
      float value_val = min_value_ + value_step * i;
      snprintf(label_buffer, sizeof(label_buffer), "%.2f", value_val);
      float y_pos = bounds.y + bounds.height - (bounds.height * i / 5);
      DrawText(label_buffer, bounds.x - 35, y_pos - 5, 8, BLACK);
    }
  }
}

void ChartsWidget::drawSignal(const Rectangle& bounds) {
  if (data_points_.size() < 2) return;
  
  // Draw the signal as a line
  for (size_t i = 1; i < data_points_.size(); i++) {
    const auto& pt1 = data_points_[i-1];
    const auto& pt2 = data_points_[i];
    
    // Convert data coordinates to screen coordinates
    float x1 = bounds.x + ((pt1.time - min_time_) / (max_time_ - min_time_)) * bounds.width;
    float y1 = bounds.y + bounds.height - ((pt1.value - min_value_) / (max_value_ - min_value_)) * bounds.height;
    float x2 = bounds.x + ((pt2.time - min_time_) / (max_time_ - min_time_)) * bounds.width;
    float y2 = bounds.y + bounds.height - ((pt2.value - min_value_) / (max_value_ - min_value_)) * bounds.height;
    
    // Only draw if points are within bounds
    if (x1 >= bounds.x && x1 <= bounds.x + bounds.width &&
        x2 >= bounds.x && x2 <= bounds.x + bounds.width) {
      DrawLine(x1, y1, x2, y2, BLUE);
    }
  }
  
  // Highlight the last point
  if (!data_points_.empty()) {
    const auto& last_pt = data_points_.back();
    float x = bounds.x + ((last_pt.time - min_time_) / (max_time_ - min_time_)) * bounds.width;
    float y = bounds.y + bounds.height - ((last_pt.value - min_value_) / (max_value_ - min_value_)) * bounds.height;
    DrawCircle(x, y, 3, RED);
  }
}

void ChartsWidget::handleInput() {
  // Handle mouse input for interaction
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    Vector2 mouse_pos = GetMousePosition();
    if (CheckCollisionPointRec(mouse_pos, bounds_)) {
      // Check if click is in chart area
      Rectangle chart_area = {
        bounds_.x + 60,
        bounds_.y + 20,
        bounds_.width - 80,
        bounds_.height - 40
      };
      
      if (CheckCollisionPointRec(mouse_pos, chart_area)) {
        // Start potential pan operation
        last_mouse_pos = mouse_pos;
        is_panning = true;
      }
    }
  }
  
  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && is_panning) {
    Vector2 current_pos = GetMousePosition();
    // Calculate drag delta
    float dx = current_pos.x - last_mouse_pos.x;
    float dy = current_pos.y - last_mouse_pos.y;
    
    // Update offsets based on drag
    time_offset_ += dx * (max_time_ - min_time_) / chart_area.width;
    value_offset_ -= dy * (max_value_ - min_value_) / chart_area.height;
    
    last_mouse_pos = current_pos;
  } else if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
    is_panning = false;
  }
  
  // Handle zoom with mouse wheel
  float wheel = GetMouseWheelMove();
  if (wheel != 0) {
    Vector2 mouse_pos = GetMousePosition();
    Rectangle chart_area = {
      bounds_.x + 60,
      bounds_.y + 20,
      bounds_.width - 80,
      bounds_.height - 40
    };
    
    if (CheckCollisionPointRec(mouse_pos, chart_area)) {
      // Zoom in/out based on wheel movement
      float zoom_factor = wheel > 0 ? 0.9f : 1.1f;
      time_scale_ *= zoom_factor;
      value_scale_ *= zoom_factor;
    }
  }
}

void ChartsWidget::handleMouseInteraction() {
  // Process mouse interactions
  handleInput();
}

void ChartsWidget::setSignal(const MessageId& msg_id, const std::string& signal_name) {
  current_msg_id_ = msg_id;
  current_signal_name_ = signal_name;
  
  // Update the title to reflect the signal
  title = signal_name + " - CHART (" + std::to_string(msg_id.address) + ")";
  
  // Clear previous data
  clear();
}

void ChartsWidget::setStream(AbstractStream* stream) {
  if (stream_ && connected_to_stream_) {
    // Disconnect from previous stream
    // In this implementation, we're not directly unsubscribing from the stream events
    // as the stream doesn't have a direct signal for individual CAN data values
  }

  stream_ = stream;
  connected_to_stream_ = (stream != nullptr);

  if (stream_) {
    // Connect to stream events to receive real-time data
    // We'll use the eventsMerged signal to get new data
    stream_->eventsMerged_signal.connect([this](const MessageEventsMap& events_map) {
      // Process all new events and extract signal values if we're monitoring a signal
      if (!current_signal_name_.empty()) {
        auto it = events_map.find(current_msg_id_);
        if (it != events_map.end()) {
          const auto& events = it->second;
          for (const auto* event : events) {
            // Get the DBC signal definition to properly decode the data
            auto* msg = DBCManager::instance()->msg(current_msg_id_);
            if (msg) {
              auto* sig = msg->sig(current_signal_name_);
              if (sig) {
                // Decode the signal value from the raw CAN data
                double raw_value = get_raw_value(event->dat, event->size, *sig);
                double scaled_value = raw_value * sig->factor + sig->offset;

                // Add data point with the timestamp
                double timestamp = (event->mono_time - can->beginMonoTime()) / 1e9;
                addDataPoint(timestamp, scaled_value);
              }
            }
          }
        }
      }
    });
  }
}

void ChartsWidget::addDataPoint(double time, double value) {
  // Add the data point
  data_points_.emplace_back(time, value);
  
  // Limit the number of data points to prevent memory issues
  const size_t max_points = 10000;  // Adjust as needed
  if (data_points_.size() > max_points) {
    // Remove old points (first 10% of the buffer)
    size_t remove_count = max_points / 10;
    data_points_.erase(data_points_.begin(), data_points_.begin() + remove_count);
  }
}

void ChartsWidget::clear() {
  data_points_.clear();
  
  // Default range
  min_time_ = 0.0;
  max_time_ = 10.0;
  min_value_ = -1.0;
  max_value_ = 1.0;
  
  // Reset view settings
  time_offset_ = 0.0;
  value_offset_ = 0.0;
  time_scale_ = 1.0;
  value_scale_ = 1.0;
}

void ChartsWidget::setTitle(const std::string& new_title) {
  title = new_title;
}

void ChartsWidget::updateDataRange() {
  if (data_points_.empty()) return;
  
  // Calculate min/max values for auto-scaling
  min_time_ = data_points_.front().time;
  max_time_ = data_points_.back().time;
  min_value_ = data_points_[0].value;
  max_value_ = data_points_[0].value;
  
  for (const auto& pt : data_points_) {
    if (pt.time < min_time_) min_time_ = pt.time;
    if (pt.time > max_time_) max_time_ = pt.time;
    if (pt.value < min_value_) min_value_ = pt.value;
    if (pt.value > max_value_) max_value_ = pt.value;
  }
  
  // Add some padding
  double time_padding = (max_time_ - min_time_) * 0.05;
  double value_padding = (max_value_ - min_value_) * 0.05;
  
  if (time_padding == 0) time_padding = 0.1;  // Prevent division by zero
  if (value_padding == 0) value_padding = 0.1;
  
  min_time_ -= time_padding;
  max_time_ += time_padding;
  min_value_ -= value_padding;
  max_value_ += value_padding;
  
  // Ensure non-zero ranges
  if (max_time_ == min_time_) {
    max_time_ = min_time_ + 1.0;
  }
  if (max_value_ == min_value_) {
    max_value_ = min_value_ + 1.0;
  }
}