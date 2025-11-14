#include "raylib_settings.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <string>

// Settings implementation without QT dependencies
Settings settings;

// Convert settings to/from string representation for persistence
std::map<std::string, std::string> loadSettings() {
  std::map<std::string, std::string> config;
  std::ifstream file("cabana_settings.cfg");
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      size_t pos = line.find('=');
      if (pos != std::string::npos) {
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        config[key] = value;
      }
    }
    file.close();
  }
  return config;
}

void saveSettings(const std::map<std::string, std::string>& config) {
  std::ofstream file("cabana_settings.cfg");
  if (file.is_open()) {
    for (const auto& pair : config) {
      file << pair.first << "=" << pair.second << std::endl;
    }
    file.close();
  }
}

Settings::Settings() {
  // Load settings from file
  auto config = loadSettings();
  
  // Set defaults if not found
  absolute_time = (config.count("absolute_time") > 0) ? (config["absolute_time"] == "1") : false;
  fps = (config.count("fps") > 0) ? std::stoi(config["fps"]) : 10;
  max_cached_minutes = (config.count("max_cached_minutes") > 0) ? std::stoi(config["max_cached_minutes"]) : 30;
  chart_height = (config.count("chart_height") > 0) ? std::stoi(config["chart_height"]) : 200;
  chart_column_count = (config.count("chart_column_count") > 0) ? std::stoi(config["chart_column_count"]) : 1;
  chart_range = (config.count("chart_range") > 0) ? std::stoi(config["chart_range"]) : 180; // 3 minutes
  chart_series_type = (config.count("chart_series_type") > 0) ? std::stoi(config["chart_series_type"]) : 0;
  theme = (config.count("theme") > 0) ? std::stoi(config["theme"]) : 0;
  sparkline_range = (config.count("sparkline_range") > 0) ? std::stoi(config["sparkline_range"]) : 15; // 15 seconds
  multiple_lines_hex = (config.count("multiple_lines_hex") > 0) ? (config["multiple_lines_hex"] == "1") : false;
  log_livestream = (config.count("log_livestream") > 0) ? (config["log_livestream"] == "1") : true;
  suppress_defined_signals = (config.count("suppress_defined_signals") > 0) ? (config["suppress_defined_signals"] == "1") : false;
  
  log_path = (config.count("log_path") > 0) ? config["log_path"] : "";
  last_dir = (config.count("last_dir") > 0) ? config["last_dir"] : "";
  last_route_dir = (config.count("last_route_dir") > 0) ? config["last_route_dir"] : "";

  video_splitter_state = (config.count("video_splitter_state") > 0) ? config["video_splitter_state"] : "";
  window_state = (config.count("window_state") > 0) ? config["window_state"] : "";
  message_header_state = (config.count("message_header_state") > 0) ? config["message_header_state"] : "";
  
  std::string recent_files_str = (config.count("recent_files") > 0) ? config["recent_files"] : "";
  // Parse recent files from a semicolon-separated string
  if (!recent_files_str.empty()) {
    std::istringstream iss(recent_files_str);
    std::string file;
    while (std::getline(iss, file, ';')) {
      recent_files.push_back(file);
    }
  }
  
  // Default drag direction is MsbFirst
  std::string drag_dir_str = (config.count("drag_direction") > 0) ? config["drag_direction"] : "0";
  int drag_dir_val = std::stoi(drag_dir_str);
  drag_direction = static_cast<DragDirection>(drag_dir_val);
}

Settings::~Settings() {
  // Save settings to file
  std::map<std::string, std::string> config;
  config["absolute_time"] = absolute_time ? "1" : "0";
  config["fps"] = std::to_string(fps);
  config["max_cached_minutes"] = std::to_string(max_cached_minutes);
  config["chart_height"] = std::to_string(chart_height);
  config["chart_column_count"] = std::to_string(chart_column_count);
  config["chart_range"] = std::to_string(chart_range);
  config["chart_series_type"] = std::to_string(chart_series_type);
  config["theme"] = std::to_string(theme);
  config["sparkline_range"] = std::to_string(sparkline_range);
  config["multiple_lines_hex"] = multiple_lines_hex ? "1" : "0";
  config["log_livestream"] = log_livestream ? "1" : "0";
  config["suppress_defined_signals"] = suppress_defined_signals ? "1" : "0";
  
  config["log_path"] = log_path;
  config["last_dir"] = last_dir;
  config["last_route_dir"] = last_route_dir;

  config["video_splitter_state"] = video_splitter_state;
  config["window_state"] = window_state;
  config["message_header_state"] = message_header_state;

  // Serialize recent files as semicolon-separated string
  std::string recent_files_str;
  for (size_t i = 0; i < recent_files.size(); ++i) {
    if (i > 0) recent_files_str += ";";
    recent_files_str += recent_files[i];
  }
  config["recent_files"] = recent_files_str;
  
  config["drag_direction"] = std::to_string(static_cast<int>(drag_direction));
  
  saveSettings(config);
}