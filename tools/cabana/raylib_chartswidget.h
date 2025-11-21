#pragma once

#include <vector>
#include <string>
#include <memory>

#include "tools/cabana/streams/abstractstream.h"
#include "tools/cabana/dbc/dbcmanager.h"
#include "tools/cabana/utils/eventmanager.h"
// Define OPENPILOT_RAYLIB before including raylib to prevent enum conflicts
#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"

// Data point for signal visualization
struct SignalDataPoint {
  double time;    // time in seconds
  double value;   // signal value
  SignalDataPoint(double t, double v) : time(t), value(v) {}
};

// Signal visualization chart widget
class ChartsWidget {
public:
  ChartsWidget(void* parent = nullptr);  // parent is just for API compatibility
  ~ChartsWidget();

  void update();
  void render(const Rectangle& bounds);
  void handleInput();

  // Set the signal to visualize
  void setSignal(const MessageId& msg_id, const std::string& signal_name);
  void setStream(AbstractStream* stream);
  
  // Add data point to the chart
  void addDataPoint(double time, double value);
  
  // Clear all data
  void clear();

  // Accessor methods for UI state
  void setTitle(const std::string& title);

private:
  void drawChart(const Rectangle& bounds);
  void drawAxes(const Rectangle& bounds);
  void drawGrid(const Rectangle& bounds);
  void drawSignal(const Rectangle& bounds);
  void updateDataRange();
  void handleMouseInteraction();

  std::string title = "SIGNAL CHART";
  std::vector<SignalDataPoint> data_points_;
  
  // Range of data
  double min_time_ = 0.0;
  double max_time_ = 10.0;
  double min_value_ = -1.0;
  double max_value_ = 1.0;
  
  // Viewport settings
  double time_offset_ = 0.0;
  double value_offset_ = 0.0;
  double time_scale_ = 1.0;
  double value_scale_ = 1.0;
  
  // UI state
  Rectangle bounds_;
  bool is_visible = true;
  
  // Interaction state
  Vector2 last_mouse_pos = {0, 0};
  bool is_panning = false;
  bool is_zooming = false;
  
  // Current signal being visualized
  MessageId current_msg_id_{0, 0};
  std::string current_signal_name_;
  AbstractStream* stream_ = nullptr;
  
  // For tracking stream data
  uint64_t event_subscription_id_ = 0;
  bool connected_to_stream_ = false;
};