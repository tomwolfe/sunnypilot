#include "tools/cabana/utils/util.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>

std::string ElidedText::getElidedText(int max_width, Font font, float font_size) const {
    // Simple implementation: just truncate text if it's too long
    // In a more sophisticated implementation, we would measure text width
    if (max_width > 100) { // If we have enough space, return full text
        return original_text_;
    } else {
        // Truncate and add ellipsis
        if (original_text_.length() > 10) {
            return original_text_.substr(0, 10) + "...";
        }
        return original_text_;
    }
}

void SegmentTree::build(const std::vector<Point> &arr) {
    if (arr.empty()) return;
    
    size_ = arr.size();
    tree.resize(4 * size_);
    build_tree(arr, 1, 0, size_ - 1);
}

void SegmentTree::build_tree(const std::vector<Point> &arr, int n, int left, int right) {
    if (left == right) {
        tree[n] = {arr[left].y, arr[left].y};  // {min, max}
    } else {
        int mid = (left + right) / 2;
        build_tree(arr, 2 * n, left, mid);
        build_tree(arr, 2 * n + 1, mid + 1, right);
        tree[n] = {std::min(tree[2 * n].first, tree[2 * n + 1].first),
                   std::max(tree[2 * n].second, tree[2 * n + 1].second)};
    }
}

std::pair<double, double> SegmentTree::minmax(int left, int right) const {
    if (size_ == 0) return {0.0, 0.0};
    return get_minmax(1, 0, size_ - 1, left, right);
}

std::pair<double, double> SegmentTree::get_minmax(int n, int left, int right, int range_left, int range_right) const {
    if (right < range_left || left > range_right) {
        return {std::numeric_limits<double>::max(), std::numeric_limits<double>::min()};
    }
    if (range_left <= left && right <= range_right) {
        return tree[n];
    }
    int mid = (left + right) / 2;
    auto left_minmax = get_minmax(2 * n, left, mid, range_left, range_right);
    auto right_minmax = get_minmax(2 * n + 1, mid + 1, right, range_left, range_right);
    return {std::min(left_minmax.first, right_minmax.first),
            std::max(left_minmax.second, right_minmax.second)};
}

UnixSignalHandler::UnixSignalHandler() {
    // Initialize signal handling (platform specific code)
    // This would typically involve using signal() function for Unix signals
}

UnixSignalHandler::~UnixSignalHandler() {
    // Cleanup signal handling
}

// Utility functions implementation
namespace utils {

std::string icon(const std::string &id) {
    // Return path to icon file based on id
    return "assets/" + id + ".png";  // This is a simplified approach
}

bool isDarkTheme() {
    // Return if the current theme is dark
    return settings.theme == 1; // Assuming 1 means dark theme
}

void setTheme(int theme) {
    settings.theme = theme;
}

std::string formatSeconds(double sec, bool include_milliseconds, bool absolute_time) {
    int hours = (int)(sec / 3600);
    int minutes = ((int)(sec / 60)) % 60;
    int seconds_int = ((int)sec) % 60;
    double milliseconds = (sec - (int)sec) * 1000;
    
    std::ostringstream ss;
    if (hours > 0) {
        ss << std::setfill('0') << std::setw(2) << hours << ":";
    }
    
    ss << std::setfill('0') << std::setw(2) << minutes << ":"
       << std::setfill('0') << std::setw(2) << seconds_int;
    
    if (include_milliseconds) {
        ss << "." << std::setfill('0') << std::setw(3) << (int)milliseconds;
    }
    
    return ss.str();
}

std::string toHex(const std::vector<uint8_t> &dat, char separator) {
    std::string result;
    for (size_t i = 0; i < dat.size(); i++) {
        if (i > 0 && separator != '\0') {
            result += separator;
        }
        char buffer[4];
        snprintf(buffer, sizeof(buffer), "%02X", dat[i]);
        result += buffer;
    }
    return result;
}

} // namespace utils

int num_decimals(double num) {
    // Convert to string and count digits after decimal point
    std::string str = std::to_string(num);
    size_t pos = str.find('.');
    if (pos != std::string::npos) {
        return str.length() - pos - 1;
    }
    return 0;
}

std::string signalToolTip(const cabana::Signal *sig) {
    if (!sig) return "";
    
    std::ostringstream ss;
    ss << "Signal: " << sig->name << "\n"
       << "Start Bit: " << sig->start_bit << "\n"
       << "Size: " << sig->size << " bits\n"
       << "Factor: " << sig->factor << "\n"
       << "Offset: " << sig->offset << "\n"
       << "Type: " << (sig->is_signed ? "Signed" : "Unsigned");
    
    return ss.str();
}

std::string toHexString(int value) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "0x%02X", value);
    return buffer;
}