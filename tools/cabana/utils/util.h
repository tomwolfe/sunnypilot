#pragma once

#include <array>
#include <cmath>
#include <vector>
#include <utility>
#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "tools/cabana/dbc/dbc.h"
#include "tools/cabana/raylib_settings.h"

// Raylib-compatible utility functions and classes

class LogSlider {
public:
  LogSlider(double factor, bool orientation, void *parent = nullptr) : factor(factor) {}

  void setRange(double min, double max) {
    log_min = factor * std::log10(min);
    log_max = factor * std::log10(max);
  }
  double value() const {
    double ratio = (current_value - min_value) / (max_value - min_value);
    double log_v = log_min + (log_max - log_min) * ratio;
    return std::round(std::pow(10, log_v / factor));
  }
  void setValue(double v) {
    current_value = v;
  }

private:
  double factor, log_min = 0, log_max = 1;
  double min_value = 0, max_value = 100;
  double current_value = 50;
};

// Basic segment tree implementation for raylib
class SegmentTree {
public:
  SegmentTree() = default;
  void build(const std::vector<std::pair<double, double>> &arr);
  inline std::pair<double, double> minmax(int left, int right) const { return get_minmax(1, 0, size - 1, left, right); }

private:
  std::pair<double, double> get_minmax(int n, int left, int right, int range_left, int range_right) const;
  void build_tree(const std::vector<std::pair<double, double>> &arr, int n, int left, int right);
  std::vector<std::pair<double, double>> tree;
  int size = 0;
};

// Simple color validation utility
bool isValidColor(const std::string &color_str) {
  // Check if it matches hex format (#RRGGBB or #RGB)
  if (color_str.empty()) return false;
  if (color_str[0] != '#') return false;

  size_t expected_length = (color_str.length() == 4) ? 4 : 7;  // #RGB or #RRGGBB
  if (color_str.length() != expected_length) return false;

  for (size_t i = 1; i < color_str.length(); ++i) {
    char c = color_str[i];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) {
      return false;
    }
  }
  return true;
}

// Utility for hex string formatting
inline std::string toHexString(int value) {
  std::stringstream ss;
  ss << "0x" << std::setfill('0') << std::setw(2) << std::uppercase << std::hex << value;
  return ss.str();
}

namespace utils {

bool isDarkTheme();
void setTheme(int theme);
std::string formatSeconds(double sec, bool include_milliseconds = false, bool absolute_time = false);
inline std::string toHex(const std::vector<uint8_t> &dat, char separator = '\0') {
  std::stringstream ss;
  for (size_t i = 0; i < dat.size(); ++i) {
    if (i > 0 && separator != '\0') ss << separator;
    ss << std::setfill('0') << std::setw(2) << std::uppercase << std::hex << static_cast<int>(dat[i]);
  }
  return ss.str();
}

}

// Unix signal handling for Raylib (simplified)
class UnixSignalHandler {
public:
  UnixSignalHandler();
  ~UnixSignalHandler();
  static void signalHandler(int s);

private:
  static void handleSigTerm();

  // Static members for signal handling
  static int sig_fd[2];
};

int num_decimals(double num);
std::string signalToolTip(const cabana::Signal *sig);
void initApp(int argc, char *argv[], bool disable_hidpi = true);
