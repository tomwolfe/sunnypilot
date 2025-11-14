#pragma once

#include <string>
#include <vector>

#define LIGHT_THEME 1
#define DARK_THEME 2

// Simple signal/slot mechanism without Qt
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

// Placeholder for settings dialog in Raylib environment
class SettingsDlg {
public:
  SettingsDlg(void* parent = nullptr);
  void save();
  
  // Placeholder fields (not actual UI elements in Raylib)
  int* fps = nullptr;
  int* cached_minutes = nullptr;
  int* chart_height = nullptr;
  int* chart_series_type = nullptr; 
  int* theme = nullptr;
  bool log_livestream = false;
  std::string log_path;
  int* drag_direction = nullptr;
};

extern Settings settings;