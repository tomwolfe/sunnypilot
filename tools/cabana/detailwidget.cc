#include "tools/cabana/detailwidget.h"
#include <sstream>

// For now, we'll provide a minimal implementation since the file was renamed
// The actual implementation would be more complete in a full migration

DetailWidget::DetailWidget(void* charts_widget, void* parent) 
    : charts_(charts_widget), parent_(parent) {
    // Initialize UI components
    binary_view_ = std::make_unique<BinaryView>(this);
    signal_view_ = std::make_unique<SignalView>(static_cast<ChartsWidget*>(charts_), this);
    history_log_ = std::make_unique<LogsWidget>(this);
    
    // Initialize with default values
    msg_details_.address = 0;
    msg_details_.size = 0;
}

DetailWidget::~DetailWidget() = default;

void DetailWidget::update() {
    if (binary_view_) binary_view_->update();
    if (signal_view_) signal_view_->update();
    if (history_log_) history_log_->update();
    
    updateState();
}

void DetailWidget::render(const Rectangle& bounds) {
    if (!is_visible_) return;
    
    bounds_ = bounds;
    
    // Draw panel background
    DrawRectangleRec(bounds, Color{250, 250, 250, 255}); // Light gray
    DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);
    
    // Draw header with message info
    Rectangle header_rect = {bounds.x, bounds.y, bounds.width, 30};
    DrawRectangleRec(header_rect, Color{220, 220, 220, 255});
    DrawRectangleLines(header_rect.x, header_rect.y, header_rect.width, header_rect.height, LIGHTGRAY);
    
    char header_text[128];
    snprintf(header_text, sizeof(header_text), "Message: 0x%X | Size: %d | Node: %s", 
             msg_details_.address, msg_details_.size, msg_details_.node.c_str());
    DrawText(header_text, bounds.x + 5, bounds.y + 5, 10, DARKGRAY);
    
    // Draw tab bar
    Rectangle tab_bar_rect = {bounds.x, bounds.y + 30, bounds.width, 25};
    DrawRectangleRec(tab_bar_rect, Color{200, 200, 200, 255});
    DrawRectangleLines(tab_bar_rect.x, tab_bar_rect.y, tab_bar_rect.width, tab_bar_rect.height, LIGHTGRAY);
    
    const char* tab_names[] = {"Binary View", "Signal View", "History Log"};
    float tab_width = bounds.width / 3.0f;
    for (int i = 0; i < 3; i++) {
        Rectangle tab_rect = {bounds.x + i * tab_width, bounds.y + 30, tab_width, 25};
        Color tab_color = (i == active_tab_) ? Color{150, 150, 255, 255} : Color{200, 200, 200, 255};
        DrawRectangleRec(tab_rect, tab_color);
        DrawRectangleLines(tab_rect.x, tab_rect.y, tab_rect.width, tab_rect.height, LIGHTGRAY);
        DrawText(tab_names[i], tab_rect.x + 5, tab_rect.y + 5, 10, DARKGRAY);
    }
    
    // Draw content based on active tab
    Rectangle content_rect = {bounds.x, bounds.y + 55, bounds.width, bounds.height - 55};
    
    switch (active_tab_) {
        case 0: // Binary View
            if (binary_view_) {
                binary_view_->render(content_rect);
            }
            break;
        case 1: // Signal View
            if (signal_view_) {
                signal_view_->render(content_rect);
            }
            break;
        case 2: // History Log
            if (history_log_) {
                history_log_->render(content_rect);
            }
            break;
    }
}

void DetailWidget::handleInput() {
    Vector2 mouse_pos = GetMousePosition();
    
    // Handle tab selection
    Rectangle tab_bar_rect = {bounds_.x, bounds_.y + 30, bounds_.width, 25};
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mouse_pos, tab_bar_rect)) {
        float tab_width = bounds_.width / 3.0f;
        active_tab_ = static_cast<int>((mouse_pos.x - bounds_.x) / tab_width);
        if (active_tab_ > 2) active_tab_ = 2;
    }
    
    // Handle input for active tab
    switch (active_tab_) {
        case 0: // Binary View
            if (binary_view_) {
                binary_view_->handleInput();
            }
            break;
        case 1: // Signal View
            if (signal_view_) {
                signal_view_->handleInput();
            }
            break;
        case 2: // History Log
            if (history_log_) {
                history_log_->handleInput();
            }
            break;
    }
}

void DetailWidget::setMessage(const MessageId &message_id) {
    msg_id_ = message_id;
    
    // Update message details
    msg_details_.msg_id = message_id;
    msg_details_.address = message_id.address;
    msg_details_.node = "Node_" + std::to_string(message_id.src);
    msg_details_.size = 8; // Default size
    
    // Update child components
    if (binary_view_) {
        binary_view_->setMessage(message_id);
    }
    if (signal_view_) {
        signal_view_->setMessage(message_id);
    }
    
    // In a real implementation, we would load actual message details
}

void DetailWidget::refresh() {
    updateState();
}

void DetailWidget::createToolBar() {
    // In a Raylib implementation, toolbar would be drawn in the render function
}

void DetailWidget::updateState() {
    // Update internal state based on data changes
}

// CenterWidget implementation
CenterWidget::CenterWidget(void* parent) {
    detail_widget_ = std::make_unique<DetailWidget>(nullptr, parent);
    show_welcome_ = true;
}

CenterWidget::~CenterWidget() = default;

void CenterWidget::update() {
    if (detail_widget_ && !show_welcome_) {
        detail_widget_->update();
    }
}

void CenterWidget::render(const Rectangle& bounds) {
    if (!is_visible_) return;
    
    bounds_ = bounds;
    
    if (show_welcome_) {
        // Draw welcome screen
        DrawRectangleRec(bounds, RAYWHITE);
        DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);
        
        const char* welcome_text = "Welcome to Cabana\n\nSelect a CAN message to view details";
        int text_width = MeasureText(welcome_text, 20);
        int text_x = bounds.x + (bounds.width - text_width) / 2;
        int text_y = bounds.y + bounds.height / 2 - 30;
        DrawText(welcome_text, text_x, text_y, 20, DARKGRAY);
    } else if (detail_widget_) {
        detail_widget_->render(bounds);
    }
}

void CenterWidget::handleInput() {
    if (show_welcome_) {
        // Handle input for welcome screen if needed
    } else if (detail_widget_) {
        detail_widget_->handleInput();
    }
}

void CenterWidget::setMessage(const MessageId &msg_id) {
    current_msg_id_ = msg_id;
    show_welcome_ = false;
    if (detail_widget_) {
        detail_widget_->setMessage(msg_id);
    }
}

void CenterWidget::clear() {
    show_welcome_ = true;
    current_msg_id_ = MessageId{}; // Reset to default
}