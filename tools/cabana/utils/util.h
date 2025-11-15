#pragma once

#include <array>
#include <cmath>
#include <vector>
#include <string>
#include <utility>

// Define OPENPILOT_RAYLIB before including raylib to prevent enum conflicts
#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"
#include "tools/cabana/dbc/dbc.h"
#include "tools/cabana/raylib_settings.h"

// Data structure for segment tree (since we can't use QPointF)
struct Point {
    double x, y;
};

// Replacement for ElidedLabel functionality
class ElidedText {
public:
    ElidedText() = default;
    ElidedText(const std::string &text) : original_text_(text) {}
    
    void setText(const std::string &text) { original_text_ = text; }
    std::string getText() const { return original_text_; }
    std::string getElidedText(int max_width, Font font, float font_size) const;
    
private:
    std::string original_text_;
};

// Simple slider implementation for logarithmic scale
class LogSlider {
public:
    LogSlider(double factor) : factor_(factor), min_val_(0.0), max_val_(100.0), current_val_(50.0) {}
    
    void setRange(double min, double max) {
        min_val_ = min;
        max_val_ = max;
        // Clamp current value to new range
        if (current_val_ < min_val_) current_val_ = min_val_;
        if (current_val_ > max_val_) current_val_ = max_val_;
    }
    
    double getValue() const {
        return current_val_;
    }
    
    void setValue(double v) {
        if (v < min_val_) v = min_val_;
        if (v > max_val_) v = max_val_;
        current_val_ = v;
    }
    
    double factor() const { return factor_; }

private:
    double factor_, min_val_, max_val_, current_val_;
};

// Segment Tree for efficient range min/max queries
class SegmentTree {
public:
    SegmentTree() = default;
    void build(const std::vector<Point> &arr);
    std::pair<double, double> minmax(int left, int right) const;

private:
    std::pair<double, double> get_minmax(int n, int left, int right, int range_left, int range_right) const;
    void build_tree(const std::vector<Point> &arr, int n, int left, int right);
    std::vector<std::pair<double, double>> tree;
    int size_ = 0;
};

// Unix Signal Handler functionality (simplified)
class UnixSignalHandler {
public:
    UnixSignalHandler();
    ~UnixSignalHandler();
    
    // Signal handling functionality would be implemented using platform-specific methods
};

// Utility functions namespace
namespace utils {

std::string icon(const std::string &id);  // For handling icons
bool isDarkTheme();
void setTheme(int theme);
std::string formatSeconds(double sec, bool include_milliseconds = false, bool absolute_time = false);
std::string toHex(const std::vector<uint8_t> &dat, char separator = '\0');

} // namespace utils

int num_decimals(double num);
std::string signalToolTip(const cabana::Signal *sig);
std::string toHexString(int value);