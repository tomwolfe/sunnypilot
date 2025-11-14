#pragma once

#include <vector>
#include <string>
#include <memory>

#include "raylib.h"
#include "tools/cabana/streams/abstractstream.h"

// Data structure for log entries
struct LogEntry {
    double timestamp;
    std::string message;
    std::string level;  // "INFO", "WARNING", "ERROR", etc.
    MessageId msg_id;
};

// Raylib-based LogsWidget
class LogsWidget {
public:
    LogsWidget(void* parent = nullptr);
    ~LogsWidget();
    
    void update();
    void render(const Rectangle& bounds);
    void handleInput();
    
    void addLogEntry(const LogEntry& entry);
    void addLogEntries(const std::vector<LogEntry>& entries);
    void clearLogs();
    void filterByLevel(const std::string& level);
    
    // Accessor methods
    const std::vector<LogEntry>& getLogEntries() const { return log_entries_; }
    void setVisible(bool visible) { is_visible_ = visible; }
    bool getVisible() const { return is_visible_; }
    
    // UI state
    std::function<void(const LogEntry&)> onLogEntryClicked;

private:
    void drawLogEntry(int index, float y_pos, const Rectangle& bounds, const LogEntry& entry);
    void handleScrolling();
    
    std::vector<LogEntry> log_entries_;
    std::vector<LogEntry> filtered_entries_;
    std::string current_filter_;
    
    // UI State
    Rectangle bounds_;
    bool is_visible_ = true;
    
    float scroll_offset_ = 0.0f;
    int selected_entry_index_ = -1;
    
    // Input state
    Vector2 last_mouse_pos_ = {0, 0};
    bool is_dragging_ = false;
    
    // Color coding for different log levels
    Color getColorForLevel(const std::string& level) const;
};