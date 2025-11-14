#include "tools/cabana/raylib_historylog.h"

LogsWidget::LogsWidget(void* parent) {
    // Initialize with default values
}

LogsWidget::~LogsWidget() = default;

void LogsWidget::update() {
    // Apply filter if needed
    if (current_filter_.empty()) {
        filtered_entries_ = log_entries_;
    } else {
        filtered_entries_.clear();
        for (const auto& entry : log_entries_) {
            if (entry.level.find(current_filter_) != std::string::npos ||
                entry.message.find(current_filter_) != std::string::npos) {
                filtered_entries_.push_back(entry);
            }
        }
    }
    
    handleScrolling();
}

void LogsWidget::render(const Rectangle& bounds) {
    if (!is_visible_) return;
    
    bounds_ = bounds;
    
    // Draw panel background
    DrawRectangleRec(bounds, Color{245, 245, 245, 255}); // Light gray
    DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);
    
    // Draw header
    Rectangle header_rect = {bounds.x, bounds.y, bounds.width, 20};
    DrawRectangleRec(header_rect, Color{220, 220, 220, 255});
    DrawRectangleLines(header_rect.x, header_rect.y, header_rect.width, header_rect.height, LIGHTGRAY);
    DrawText("LOG HISTORY", bounds.x + 5, bounds.y + 3, 10, DARKGRAY);
    
    // Draw filter input placeholder
    Rectangle filter_rect = {bounds.x + 80, bounds.y + 2, 150, 16};
    DrawRectangleLines(filter_rect.x, filter_rect.y, filter_rect.width, filter_rect.height, GRAY);
    DrawText("Filter logs...", filter_rect.x + 5, filter_rect.y + 2, 8, GRAY);
    
    // Draw logs
    float yPos = bounds.y + 22 + scroll_offset_;
    int displayedCount = 0;
    const int maxDisplay = static_cast<int>((bounds.height - 25) / 18);
    
    for (size_t i = 0; i < filtered_entries_.size() && displayedCount < maxDisplay; ++i) {
        if (yPos > bounds.y + 22 && yPos < bounds.y + bounds.height) {
            drawLogEntry(i, yPos, bounds, filtered_entries_[i]);
            yPos += 18;
            displayedCount++;
        } else if (yPos < bounds.y + 22) {
            yPos += 18;
        }
    }
    
    // Draw scroll indicator
    if (!filtered_entries_.empty()) {
        float maxScroll = -static_cast<int>(filtered_entries_.size()) * 18 + bounds.height - 25;
        if (maxScroll < 0) {  // Only draw scrollbar if content exceeds view
            float scroll_ratio = (scroll_offset_ - maxScroll) / (-maxScroll);
            float scrollbar_height = bounds.height * 0.2f;
            if (scrollbar_height < 20.0f) scrollbar_height = 20.0f;
            if (scrollbar_height > bounds.height - 25) scrollbar_height = bounds.height - 25;
            
            float scrollbar_y = bounds.y + 22 + (bounds.height - 47) * (1.0f - scroll_ratio);
            
            DrawRectangle(bounds.x + bounds.width - 15, scrollbar_y, 10, scrollbar_height, GRAY);
        }
    }
}

void LogsWidget::drawLogEntry(int index, float yPos, const Rectangle& bounds, const LogEntry& entry) {
    Color textColor = (index == selected_entry_index_) ? BLUE : DARKGRAY;
    Color bgColor = (index == selected_entry_index_) ? Color{200, 200, 255, 255} : RAYWHITE;
    Color levelColor = getColorForLevel(entry.level);

    // Draw row background if selected
    if (index == selected_entry_index_) {
        DrawRectangle(bounds.x + 1, static_cast<int>(yPos - 1), 
                     static_cast<int>(bounds.width - 2), 16, bgColor);
    }

    // Draw timestamp
    char time_str[32];
    snprintf(time_str, sizeof(time_str), "%.2f", entry.timestamp);
    DrawText(time_str, bounds.x + 5, static_cast<int>(yPos), 8, DARKGRAY);
    
    // Draw level indicator
    DrawText(entry.level.c_str(), bounds.x + 80, static_cast<int>(yPos), 8, levelColor);
    
    // Draw message
    DrawText(entry.message.c_str(), bounds.x + 130, static_cast<int>(yPos), 8, textColor);
}

void LogsWidget::handleInput() {
    Vector2 mouse_pos = GetMousePosition();
    
    // Handle log selection
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointRec(mouse_pos, bounds_)) {
            // Only in content area (below header)
            if (mouse_pos.y > bounds_.y + 22) {
                // Calculate which log was clicked based on scroll position
                float relativeY = mouse_pos.y - bounds_.y - 22; // Adjust for header
                int logIndex = static_cast<int>((relativeY - scroll_offset_) / 18); // Approximate log height
                
                if (logIndex >= 0 && logIndex < static_cast<int>(filtered_entries_.size())) {
                    selected_entry_index_ = logIndex;
                    if (onLogEntryClicked) {
                        onLogEntryClicked(filtered_entries_[logIndex]);
                    }
                }
            }
        }
    }
    
    handleScrolling();
}

void LogsWidget::handleScrolling() {
    // Handle scrolling with mouse wheel
    float wheel_move = GetMouseWheelMove();
    if (wheel_move > 0) {
        scroll_offset_ += 36;  // Scroll up
        if (scroll_offset_ > 0) scroll_offset_ = 0;
    } else if (wheel_move < 0) {
        scroll_offset_ -= 36;  // Scroll down
        float maxScroll = -static_cast<int>(filtered_entries_.size()) * 18 + bounds_.height - 25;
        if (scroll_offset_ < maxScroll) scroll_offset_ = maxScroll;
    }
    
    // Mouse drag scrolling
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        Vector2 current_mouse = GetMousePosition();
        if (CheckCollisionPointRec(current_mouse, bounds_) && !is_dragging_) {
            if (current_mouse.y > bounds_.y + 22) { // Only in content area
                last_mouse_pos_ = current_mouse;
                is_dragging_ = true;
            }
        } else if (is_dragging_) {
            float deltaY = current_mouse.y - last_mouse_pos_.y;
            scroll_offset_ += deltaY;
            
            float maxScroll = -static_cast<int>(filtered_entries_.size()) * 18 + bounds_.height - 25;
            if (scroll_offset_ > 0) scroll_offset_ = 0;
            if (scroll_offset_ < maxScroll) scroll_offset_ = maxScroll;
            
            last_mouse_pos_ = current_mouse;
        }
    } else {
        is_dragging_ = false;
    }
}

void LogsWidget::addLogEntry(const LogEntry& entry) {
    log_entries_.push_back(entry);
    update(); // Apply filtering
}

void LogsWidget::addLogEntries(const std::vector<LogEntry>& entries) {
    log_entries_.insert(log_entries_.end(), entries.begin(), entries.end());
    update(); // Apply filtering
}

void LogsWidget::clearLogs() {
    log_entries_.clear();
    filtered_entries_.clear();
}

void LogsWidget::filterByLevel(const std::string& level) {
    current_filter_ = level;
    update();
}

Color LogsWidget::getColorForLevel(const std::string& level) const {
    if (level == "ERROR" || level == "FATAL") {
        return RED;
    } else if (level == "WARNING" || level == "WARN") {
        return ORANGE;
    } else if (level == "INFO") {
        return BLUE;
    } else if (level == "DEBUG") {
        return GRAY;
    }
    return DARKGRAY; // Default color
}