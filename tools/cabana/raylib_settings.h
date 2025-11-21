#pragma once

#include <string>
#include <vector>

#define LIGHT_THEME 1
#define DARK_THEME 2

// Settings interface using Raylib
class Settings {
public:
  enum DragDirection {
    MsbFirst,
    LsbFirst,
    AlwaysLE,
    AlwaysBE,
  };

  Settings();
  ~Settings();

  bool absolute_time = false;
  int fps = 10;
  int max_cached_minutes = 30;
  int chart_height = 200;
  int chart_column_count = 1;
  int chart_range = 3 * 60; // 3 minutes
  int chart_series_type = 0;
  int theme = 0;
  int sparkline_range = 15; // 15 seconds
  bool multiple_lines_hex = false;
  bool log_livestream = true;
  bool suppress_defined_signals = false;
  std::string log_path;
  std::string last_dir;
  std::string last_route_dir;
  std::string geometry;
  std::string video_splitter_state;
  std::string window_state;
  std::vector<std::string> recent_files;
  std::string message_header_state;
  DragDirection drag_direction = MsbFirst;
};

// Forward declaration for settings dialog
class SettingsDlg;

extern Settings settings;