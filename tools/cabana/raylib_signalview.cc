#include "raylib_signalview.h"
#include <sstream>
#include <iomanip>

SignalView::SignalView(ChartsWidget *charts_widget, void* parent) 
    : charts_widget_(charts_widget) {
    // Initialize with default values
    updateToolBar();
}

SignalView::~SignalView() = default;

void SignalView::update() {
    updateSignalList();
    handleScrolling();
    updateToolBar();
}

void SignalView::render(const Rectangle& bounds) {
    if (!is_visible_) return;
    
    bounds_ = bounds;
    
    // Draw panel background
    DrawRectangleRec(bounds, Color{245, 245, 245, 255}); // Light gray
    DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);

    // Draw toolbar
    Rectangle toolbar_rect = {bounds.x, bounds.y, bounds.width, 30};
    DrawRectangleRec(toolbar_rect, Color{230, 230, 230, 255});
    DrawRectangleLines(toolbar_rect.x, toolbar_rect.y, toolbar_rect.width, toolbar_rect.height, LIGHTGRAY);
    
    // Draw toolbar content
    DrawText(signal_count_text_.c_str(), bounds.x + 5, bounds.y + 5, 12, DARKGRAY);
    
    // Draw filter input placeholder
    Rectangle filter_rect = {bounds.x + 80, bounds.y + 5, 150, 20};
    DrawRectangleLines(filter_rect.x, filter_rect.y, filter_rect.width, filter_rect.height, GRAY);
    DrawText("Filter Signal", filter_rect.x + 5, filter_rect.y + 3, 10, GRAY);
    
    // Draw sparkline range control
    DrawText("Range:", bounds.x + bounds.width - 150, bounds.y + 5, 10, DARKGRAY);
    char range_str[32];
    snprintf(range_str, sizeof(range_str), "%ds", sparkline_range_);
    DrawText(range_str, bounds.x + bounds.width - 100, bounds.y + 5, 10, DARKGRAY);
    
    // Draw column headers
    Rectangle header_rect = {bounds.x, bounds.y + 30, bounds.width, 20};
    DrawRectangleRec(header_rect, Color{220, 220, 220, 255});
    DrawRectangleLines(header_rect.x, header_rect.y, header_rect.width, header_rect.height, GRAY);
    
    DrawText("Name", bounds.x + 10, bounds.y + 32, 10, DARKGRAY);
    DrawText("Value", bounds.x + bounds.width - 100, bounds.y + 32, 10, DARKGRAY);
    
    // Draw signals
    float yPos = bounds.y + 50 + scroll_offset_;
    int displayedCount = 0;
    const int maxDisplay = static_cast<int>((bounds.height - 55) / 20);
    
    for (size_t i = 0; i < signals_.size() && displayedCount < maxDisplay; ++i) {
        if (yPos > bounds.y + 50 && yPos < bounds.y + bounds.height) {
            drawSignalRow(i, yPos, bounds, signals_[i]);
            yPos += 20;
            displayedCount++;
        } else if (yPos < bounds.y + 50) {
            yPos += 20;
        }
    }
    
    // Draw scroll indicator
    if (!signals_.empty()) {
        float maxScroll = -static_cast<int>(signals_.size()) * 20 + bounds.height - 55;
        if (maxScroll < 0) {  // Only draw scrollbar if content exceeds view
            float scroll_ratio = (scroll_offset_ - maxScroll) / (-maxScroll);
            float scrollbar_height = bounds.height * 0.2f;
            if (scrollbar_height < 20.0f) scrollbar_height = 20.0f;
            if (scrollbar_height > bounds.height - 55) scrollbar_height = bounds.height - 55;
            
            float scrollbar_y = bounds.y + 50 + (bounds.height - 105) * (1.0f - scroll_ratio);
            
            DrawRectangle(bounds.x + bounds.width - 15, scrollbar_y, 10, scrollbar_height, GRAY);
        }
    }
}

void SignalView::drawSignalRow(int index, float yPos, const Rectangle& bounds, const SignalInfo& signal) {
    Color textColor = (index == selected_signal_index_) ? BLUE : DARKGRAY;
    Color bgColor = (index == selected_signal_index_) ? Color{200, 200, 255, 255} : RAYWHITE;

    // Draw row background if selected
    if (index == selected_signal_index_) {
        DrawRectangle(bounds.x + 1, static_cast<int>(yPos - 1), 
                     static_cast<int>(bounds.width - 2), 18, bgColor);
    }

    // Draw signal name
    DrawText(signal.name.c_str(), bounds.x + 10, static_cast<int>(yPos), 10, textColor);
    
    // Draw signal value (for demonstration)
    DrawText("0.00", bounds.x + bounds.width - 80, static_cast<int>(yPos), 10, textColor);
    
    // Draw signal type indicator if multiplexed
    if (signal.type != "Normal Signal") {
        Rectangle indicator_rect = {bounds.x + bounds.width - 120, static_cast<int>(yPos), 30, 15};
        DrawRectangleRec(indicator_rect, GRAY);
        DrawText(signal.type == "Multiplexor Signal" ? "M" : "m", 
                 indicator_rect.x + 10, indicator_rect.y + 1, 8, WHITE);
    }
}

void SignalView::handleInput() {
    Vector2 mouse_pos = GetMousePosition();
    
    // Handle signal selection
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointRec(mouse_pos, bounds_)) {
            // Only in content area (below toolbar and headers)
            if (mouse_pos.y > bounds_.y + 50) {
                // Calculate which signal was clicked based on scroll position
                float relativeY = mouse_pos.y - bounds_.y - 50; // Adjust for toolbar and headers
                int signalIndex = static_cast<int>((relativeY - scroll_offset_) / 20); // Approximate signal height
                
                if (signalIndex >= 0 && signalIndex < static_cast<int>(signals_.size())) {
                    selected_signal_index_ = signalIndex;
                    // In a full implementation, we would emit a signal here
                }
            }
        }
    }
    
    handleScrolling();
}

void SignalView::handleScrolling() {
    // Handle scrolling with mouse wheel
    float wheel_move = GetMouseWheelMove();
    if (wheel_move > 0) {
        scroll_offset_ += 40;  // Scroll up
        if (scroll_offset_ > 0) scroll_offset_ = 0;
    } else if (wheel_move < 0) {
        scroll_offset_ -= 40;  // Scroll down
        float maxScroll = -static_cast<int>(signals_.size()) * 20 + bounds_.height - 55;
        if (scroll_offset_ < maxScroll) scroll_offset_ = maxScroll;
    }
    
    // Mouse drag scrolling
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        Vector2 current_mouse = GetMousePosition();
        if (CheckCollisionPointRec(current_mouse, bounds_) && !is_dragging_) {
            if (current_mouse.y > bounds_.y + 50) { // Only in content area
                last_mouse_pos_ = current_mouse;
                is_dragging_ = true;
            }
        } else if (is_dragging_) {
            float deltaY = current_mouse.y - last_mouse_pos_.y;
            scroll_offset_ += deltaY;
            
            float maxScroll = -static_cast<int>(signals_.size()) * 20 + bounds_.height - 55;
            if (scroll_offset_ > 0) scroll_offset_ = 0;
            if (scroll_offset_ < maxScroll) scroll_offset_ = maxScroll;
            
            last_mouse_pos_ = current_mouse;
        }
    } else {
        is_dragging_ = false;
    }
}

void SignalView::setMessage(const MessageId &id) {
    current_msg_id_ = id;
    filter_text_.clear();
    updateSignalList();
}

void SignalView::setFilter(const std::string &filter) {
    filter_text_ = filter;
    updateSignalList();
}

void SignalView::selectSignal(const cabana::Signal *sig, bool expand) {
    // In a real implementation, this would find and select the signal
    // For now, we'll just select the first matching signal by name if possible
    for (size_t i = 0; i < signals_.size(); ++i) {
        if (signals_[i].name == sig->name) {
            selected_signal_index_ = i;
            // Scroll to make the selected signal visible
            float item_pos = i * 20 + scroll_offset_;
            if (item_pos < bounds_.y + 50) {
                scroll_offset_ = bounds_.y + 50 - i * 20;
            } else if (item_pos > bounds_.y + bounds_.height - 20) {
                scroll_offset_ = bounds_.y + bounds_.height - 20 - i * 20;
            }
            break;
        }
    }
}

void SignalView::signalHovered(const cabana::Signal *sig) {
    // Highlight the signal in the UI
    // In a real implementation, this would update the UI to show which signal is hovered
}

void SignalView::updateState() {
    // Update internal state based on data changes
    updateSignalList();
}

void SignalView::updateSignalList() {
    // Clear existing signals
    signals_.clear();
    
    // In a real implementation, this would get signals from the DBC manager for the current message
    // For now, we'll add some sample signals
    if (current_msg_id_.address != 0) {  // Only add sample data if a message is selected
        SignalInfo sig1;
        sig1.name = "STEERING_ANGLE";
        sig1.size = 16;
        sig1.receiver_nodes = "DRIVER";
        sig1.is_little_endian = true;
        sig1.is_signed = true;
        sig1.offset = 0.0;
        sig1.factor = 0.1;
        sig1.type = "Normal Signal";
        sig1.multiplex_value = -1;
        sig1.unit = "deg";
        sig1.comment = "Steering wheel angle";
        sig1.min_value = -780.0;
        sig1.max_value = 780.0;
        sig1.value_description = "";
        signals_.push_back(sig1);
        
        SignalInfo sig2;
        sig2.name = "VEHICLE_SPEED";
        sig2.size = 16;
        sig2.receiver_nodes = "DRIVER";
        sig2.is_little_endian = true;
        sig2.is_signed = false;
        sig2.offset = 0.0;
        sig2.factor = 0.01;
        sig2.type = "Normal Signal";
        sig2.multiplex_value = -1;
        sig2.unit = "m/s";
        sig2.comment = "Vehicle speed";
        sig2.min_value = 0.0;
        sig2.max_value = 150.0;
        sig2.value_description = "";
        signals_.push_back(sig2);
    }
}

void SignalView::updateToolBar() {
    signal_count_text_ = "Signals: " + std::to_string(signals_.size());
}