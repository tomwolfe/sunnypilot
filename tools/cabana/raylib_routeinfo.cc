#include "raylib_routeinfo.h"
#include <sstream>

// Global instance
RouteInfoDlg* routeInfoDlg = nullptr;

RouteInfoDlg::RouteInfoDlg(void* parent) {
    windowBounds = {300, 150, 700, 500};  // x, y, width, height
    visible = false;
    
    // Initialize table bounds
    tableBounds = {windowBounds.x + 20, windowBounds.y + 50, 660, 400};
    
    // Load initial segment info - in a real implementation this would come from the replay stream
    loadSegmentInfo();
}

RouteInfoDlg::~RouteInfoDlg() = default;

void RouteInfoDlg::show() {
    visible = true;
    // Load the latest segment info when shown
    loadSegmentInfo();
}

void RouteInfoDlg::hide() {
    visible = false;
}

void RouteInfoDlg::update() {
    if (!visible) return;

    Vector2 mousePos = GetMousePosition();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        // Check if clicked on table row
        if (CheckCollisionPointRec(mousePos, tableBounds)) {
            int row = getRowAtPosition(mousePos.y, tableBounds);
            if (row >= 0 && row < static_cast<int>(segments.size())) {
                selectedRow = row;
                
                // Callback to seek to the selected segment (each segment is 1 minute)
                if (onSeekToSegment) {
                    onSeekToSegment(row * 60.0);  // Each segment is 1 minute
                }
            }
        }
    }
    
    // Handle scrolling with mouse wheel
    float wheelMove = GetMouseWheelMove();
    if (wheelMove > 0) {
        scroll_offset_ -= 3;  // Scroll up
        if (scroll_offset_ < 0) scroll_offset_ = 0;
    } else if (wheelMove < 0) {
        scroll_offset_ += 3;  // Scroll down
        float maxScroll = std::max(0.0f, static_cast<float>(segments.size()) * 20 - (tableBounds.height - 20));
        if (scroll_offset_ > maxScroll) scroll_offset_ = maxScroll;
    }
}

void RouteInfoDlg::render(const Rectangle& parentBounds) {
    if (!visible) return;
    
    // Draw modal background
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(GRAY, 0.5f));
    
    // Draw window background
    DrawRectangleRec(windowBounds, Color{245, 245, 245, 255}); // Light gray
    DrawRectangleLines(windowBounds.x, windowBounds.y, windowBounds.width, windowBounds.height, BLACK);
    
    // Draw window title
    DrawText(("Route: " + routeName).c_str(), windowBounds.x + 10, windowBounds.y + 10, 16, BLACK);
    
    // Draw table header
    const char* headers[] = {"", "rlog", "fcam", "ecam", "dcam", "qlog", "qcam"};
    float col_width = tableBounds.width / 7.0f;
    
    for (int i = 0; i < 7; i++) {
        Rectangle header_rect = {tableBounds.x + i * col_width, tableBounds.y, col_width, 20};
        DrawRectangleRec(header_rect, Color{200, 200, 200, 255});
        DrawRectangleLines(header_rect.x, header_rect.y, header_rect.width, header_rect.height, BLACK);
        DrawText(headers[i], header_rect.x + 5, header_rect.y + 3, 10, BLACK);
    }
    
    // Draw table rows
    drawTable(tableBounds);
    
    // Add tooltip
    DrawText("Click on a row to seek to the corresponding segment.", windowBounds.x + 10, windowBounds.y + 460, 10, DARKGRAY);
}

void RouteInfoDlg::drawTable(const Rectangle& bounds) {
    float row_height = 20;
    int start_row = static_cast<int>(scroll_offset_ / row_height);
    int end_row = std::min(static_cast<int>(segments.size()), 
                          start_row + static_cast<int>(bounds.height / row_height) + 1);
    
    float y_pos = bounds.y + 20 - fmod(scroll_offset_, row_height);
    
    for (int i = start_row; i < end_row && i < static_cast<int>(segments.size()); i++) {
        // Highlight selected row
        if (i == selectedRow) {
            DrawRectangle(bounds.x, y_pos, bounds.width, row_height, Color{180, 180, 255, 255});
        }
        
        // Draw row separators
        DrawRectangleLines(bounds.x, y_pos, bounds.width, row_height, LIGHTGRAY);
        
        // Draw data in columns
        float col_width = bounds.width / 7.0f;
        
        // Column 0: Segment number
        std::string seg_num = std::to_string(segments[i].segment_number);
        DrawText(seg_num.c_str(), bounds.x + 5, y_pos + 2, 10, DARKGRAY);
        
        // Columns 1-6: Status indicators
        std::vector<std::string> statuses = {
            segments[i].rlog_status,
            segments[i].fcam_status, 
            segments[i].ecam_status,
            segments[i].dcam_status,
            segments[i].qlog_status,
            segments[i].qcam_status
        };
        
        for (int j = 0; j < 6; j++) {
            std::string status = statuses[j];
            Color statusColor = status == "Yes" ? GREEN : GRAY;
            DrawText(status.c_str(), bounds.x + (j+1) * col_width + 5, y_pos + 2, 10, statusColor);
        }
        
        y_pos += row_height;
    }
}

void RouteInfoDlg::handleInput() {
    // Already handled in update() method
}

void RouteInfoDlg::loadSegmentInfo() {
    // In a real implementation, this would get segment information from the replay stream
    // For now, adding some placeholder data
    
    segments.clear();
    
    // Example with 10 segments
    for (int i = 0; i < 10; i++) {
        SegmentInfo seg;
        seg.segment_number = i;
        seg.has_rlog = (i % 2 == 0);  // Alternate
        seg.has_fcam = true;
        seg.has_ecam = (i < 8);  // First 8 have ecam
        seg.has_dcam = (i > 2);  // After 2 have dcam
        seg.has_qlog = true;
        seg.has_qcam = (i % 3 == 0);  // Every third has qcam
        
        seg.rlog_status = seg.has_rlog ? "Yes" : "--";
        seg.fcam_status = seg.has_fcam ? "Yes" : "--";
        seg.ecam_status = seg.has_ecam ? "Yes" : "--";
        seg.dcam_status = seg.has_dcam ? "Yes" : "--";
        seg.qlog_status = seg.has_qlog ? "Yes" : "--";
        seg.qcam_status = seg.has_qcam ? "Yes" : "--";
        
        segments.push_back(seg);
    }
    
    routeName = "Example Route Name";
}

int RouteInfoDlg::getRowAtPosition(float y, const Rectangle& bounds) const {
    float row_height = 20;
    float relative_y = y - bounds.y - 20;  // Offset for header
    int row = static_cast<int>((relative_y + scroll_offset_) / row_height);
    
    if (row >= 0 && row < static_cast<int>(segments.size())) {
        return row;
    }
    return -1;
}