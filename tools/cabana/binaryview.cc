#include "tools/cabana/binaryview.h"
#include <sstream>

BinaryView::BinaryView(void* parent) {
    // Initialize with default values
    items_.resize(8 * 8); // Default for a basic 8x8 grid
    selected_row_ = -1;
    selected_col_ = -1;
}

BinaryView::~BinaryView() = default;

void BinaryView::update() {
    // Update binary view state
    // In a real implementation, this would update based on stream data
    is_message_active = false; // Update based on actual stream state
    
    // Update bit flip tracker if needed
    if (heatmap_live_mode) {
        // Update heatmap data in real-time
    }
}

void BinaryView::render(const Rectangle& bounds) {
    if (!is_visible_) return;
    
    bounds_ = bounds;
    
    // Draw panel background
    DrawRectangleRec(bounds, Color{40, 40, 40, 255}); // Dark gray
    DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);

    // Draw binary data grid
    drawBinaryData(bounds);
    
    // Draw header row
    Rectangle header = {bounds.x, bounds.y, bounds.width, 25};
    DrawRectangleRec(header, Color{60, 60, 60, 255});
    DrawRectangleLines(header.x, header.y, header.width, header.height, GRAY);
    
    // Draw column headers (0-7 for bytes)
    for (int i = 0; i < 8; i++) {
        char label[8];
        snprintf(label, sizeof(label), "B%d", i);
        float col_width = bounds.width / 8.0f;
        float x = bounds.x + i * col_width;
        DrawText(label, x + col_width/2 - MeasureText(label, 10)/2, bounds.y + 5, 10, WHITE);
        
        // Draw vertical separators
        if (i > 0) {
            DrawLine(x, bounds.y, x, bounds.y + bounds.height, DARKGRAY);
        }
    }
    
    // Draw selected cell highlight if applicable
    if (selected_row_ >= 0 && selected_col_ >= 0) {
        float col_width = bounds.width / 8.0f;
        float row_height = (bounds.height - 25) / 8.0f; // Account for header
        float x = bounds.x + selected_col_ * col_width;
        float y = bounds.y + 25 + selected_row_ * row_height;
        
        DrawRectangleLines(x, y, col_width, row_height, YELLOW);
    }
}

void BinaryView::drawBinaryData(const Rectangle& bounds) {
    // Draw the main binary data area (below header)
    Rectangle data_area = {bounds.x, bounds.y + 25, bounds.width, bounds.height - 25};
    
    int rows = 8;
    int cols = 8;
    
    float cell_width = data_area.width / cols;
    float cell_height = data_area.height / rows;
    
    for (int row = 0; row < rows && row < 8; row++) {
        for (int col = 0; col < cols && col < 8; col++) {
            int index = row * cols + col;
            if (index >= static_cast<int>(items_.size())) continue;
            
            const BinaryViewItem& item = items_[index];
            
            float x = data_area.x + col * cell_width;
            float y = data_area.y + row * cell_height;
            
            Rectangle cell = {x, y, cell_width, cell_height};
            
            // Draw cell background
            DrawRectangleRec(cell, item.bg_color);
            DrawRectangleLines(cell.x, cell.y, cell.width, cell.height, GRAY);
            
            // Draw value as hex
            char hex_str[4];
            snprintf(hex_str, sizeof(hex_str), "%02X", item.val);
            int text_width = MeasureText(hex_str, 10);
            int text_x = x + (cell_width - text_width) / 2;
            int text_y = y + (cell_height - 10) / 2;
            DrawText(hex_str, text_x, text_y, 10, WHITE);
            
            // Draw MSB/LSB indicators if applicable
            if (item.is_msb) {
                DrawText("M", x + 2, y + 2, 8, YELLOW);
            }
            if (item.is_lsb) {
                DrawText("L", x + 2, y + cell_height - 10, 8, YELLOW);
            }
        }
    }
}

void BinaryView::handleInput() {
    Vector2 mouse_pos = GetMousePosition();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        handleMouseInput(mouse_pos, bounds_);
    }
    
    // Handle hover detection
    handleMouseInput(mouse_pos, bounds_); // This will also handle hover detection
}

void BinaryView::handleMouseInput(const Vector2& mouse_pos, const Rectangle& bounds) {
    if (CheckCollisionPointRec(mouse_pos, bounds)) {
        // Only process if in data area (below header)
        if (mouse_pos.y > bounds.y + 25) {
            int col = getColumnAtPosition(mouse_pos.x, bounds);
            int row = getRowAtPosition(mouse_pos.y, bounds);
            
            if (col >= 0 && row >= 0) {
                selected_row_ = row;
                selected_col_ = col;
                
                // In a full implementation, this would trigger appropriate callbacks
                if (onSignalHovered) {
                    // Call hover callback with appropriate signal if one exists at this position
                }
            }
        }
    }
}

int BinaryView::getColumnAtPosition(float x, const Rectangle& bounds) {
    float col_width = bounds.width / 8.0f;
    int col = static_cast<int>((x - bounds.x) / col_width);
    if (col >= 0 && col < 8) {
        return col;
    }
    return -1;
}

int BinaryView::getRowAtPosition(float y, const Rectangle& bounds) {
    float row_height = (bounds.height - 25) / 8.0f; // Account for header
    int row = static_cast<int>((y - bounds.y - 25) / row_height);
    if (row >= 0 && row < 8) {
        return row;
    }
    return -1;
}

void BinaryView::setMessage(const MessageId &message_id) {
    msg_id_ = message_id;
    // In a real implementation, this would load the message data
    // For now, initialize with some sample data
    items_.clear();
    items_.resize(64); // 8x8 grid
    
    for (size_t i = 0; i < items_.size(); i++) {
        items_[i].val = i % 256; // Sample data
        items_[i].valid = true;
    }
}

void BinaryView::highlight(const cabana::Signal *sig) {
    hovered_sig = sig;
    // In a full implementation, this would update the UI to highlight the signal
}

std::vector<const cabana::Signal*> BinaryView::getOverlappingSignals() const {
    std::vector<const cabana::Signal*> overlapping;
    // In a real implementation, this would return signals that overlap with the current view
    return overlapping;
}

void BinaryView::updateState() {
    // Update the display state based on current data
    update();
}