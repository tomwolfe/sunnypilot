#include "raylib_findsimilarbits.h"
#include <sstream>
#include <algorithm>

// Global instance
FindSimilarBitsDlg* findSimilarBitsDlg = nullptr;

FindSimilarBitsDlg::FindSimilarBitsDlg(void* parent) {
    windowBounds = {250, 100, 800, 600};  // x, y, width, height
    visible = false;
    
    // Initialize UI element bounds
    searchButtonBounds = {windowBounds.x + 650, windowBounds.y + 180, 100, 30};
    tableBounds = {windowBounds.x + 20, windowBounds.y + 220, 760, 360};
    
    // Initialize with default values
    minMsgs = "100";
    
    // Initialize available buses and messages - in a real implementation these would come from the stream
    availableBuses = {0, 1, 2}; // Example buses
    availableMessages = {{"Message1", 0x123}, {"Message2", 0x456}, {"Message3", 0x789}}; // Example messages
}

FindSimilarBitsDlg::~FindSimilarBitsDlg() = default;

void FindSimilarBitsDlg::show() {
    visible = true;
}

void FindSimilarBitsDlg::hide() {
    visible = false;
}

void FindSimilarBitsDlg::update() {
    if (!visible) return;

    Vector2 mousePos = GetMousePosition();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        // Check search button
        if (CheckCollisionPointRec(mousePos, searchButtonBounds)) {
            find();
        }
        // Check table for selection
        else if (CheckCollisionPointRec(mousePos, tableBounds)) {
            int row = getRowAtPosition(mousePos.y, tableBounds);
            if (row >= 0 && row < static_cast<int>(results.size())) {
                selectedRow = row;
                
                // Double-click simulation (in real app this would be two quick clicks)
                if (GetTime() - lastClickTime < 0.5) {  // Double click threshold
                    if (onOpenMessage && selectedRow >= 0 && selectedRow < results.size()) {
                        MessageId msg_id = {.source = static_cast<uint8_t>(findBus), .address = results[selectedRow].address};
                        onOpenMessage(msg_id);
                    }
                }
                lastClickTime = GetTime();
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
        float maxScroll = std::max(0.0f, static_cast<float>(results.size()) * 20 - (tableBounds.height - 20));
        if (scroll_offset_ > maxScroll) scroll_offset_ = maxScroll;
    }
    
    // Handle bus selection - we'll use clicks on the bus display areas to change values
    Rectangle srcBusBounds = {windowBounds.x + 100, windowBounds.y + 35, 60, 20};
    Rectangle findBusBounds = {windowBounds.x + 100, windowBounds.y + 120, 60, 20};
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mousePos, srcBusBounds)) {
        // Cycle through available buses
        auto it = std::find(availableBuses.begin(), availableBuses.end(), srcBus);
        if (it != availableBuses.end()) {
            size_t idx = std::distance(availableBuses.begin(), it);
            srcBus = availableBuses[(idx + 1) % availableBuses.size()];
        } else if (!availableBuses.empty()) {
            srcBus = availableBuses[0];
        }
    }
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mousePos, findBusBounds)) {
        // Cycle through available buses
        auto it = std::find(availableBuses.begin(), availableBuses.end(), findBus);
        if (it != availableBuses.end()) {
            size_t idx = std::distance(availableBuses.begin(), it);
            findBus = availableBuses[(idx + 1) % availableBuses.size()];
        } else if (!availableBuses.empty()) {
            findBus = availableBuses[0];
        }
    }
    
    // Handle equal selection
    Rectangle equalBounds = {windowBounds.x + 180, windowBounds.y + 120, 80, 20};
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mousePos, equalBounds)) {
        equalSelection = (equalSelection + 1) % 2;  // Cycle between Yes and No
    }
}

void FindSimilarBitsDlg::render(const Rectangle& parentBounds) {
    if (!visible) return;
    
    // Draw modal background
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(GRAY, 0.5f));
    
    // Draw window background
    DrawRectangleRec(windowBounds, Color{245, 245, 245, 255}); // Light gray
    DrawRectangleLines(windowBounds.x, windowBounds.y, windowBounds.width, windowBounds.height, BLACK);
    
    // Draw window title
    DrawText("Find similar bits", windowBounds.x + 10, windowBounds.y + 5, 20, BLACK);
    
    // Draw source section
    DrawText("Find From:", windowBounds.x + 20, windowBounds.y + 10, 14, BLACK);
    
    DrawText("Bus:", windowBounds.x + 30, windowBounds.y + 35, 12, BLACK);
    Rectangle srcBusBounds = {windowBounds.x + 100, windowBounds.y + 35, 60, 20};
    DrawRectangleRec(srcBusBounds, LIGHTGRAY);
    DrawRectangleLines(srcBusBounds.x, srcBusBounds.y, srcBusBounds.width, srcBusBounds.height, BLACK);
    DrawText(std::to_string(srcBus).c_str(), srcBusBounds.x + 5, srcBusBounds.y + 3, 10, BLACK);
    
    // Draw message selection
    DrawText("Message:", windowBounds.x + 180, windowBounds.y + 35, 12, BLACK);
    Rectangle msgBounds = {windowBounds.x + 250, windowBounds.y + 35, 150, 20};
    DrawRectangleRec(msgBounds, LIGHTGRAY);
    DrawRectangleLines(msgBounds.x, msgBounds.y, msgBounds.width, msgBounds.height, BLACK);
    if (!availableMessages.empty()) {
        std::string msgName = availableMessages[0].first;  // For simplicity, showing first available
        DrawText(msgName.c_str(), msgBounds.x + 5, msgBounds.y + 3, 10, BLACK);
    }
    
    // Draw byte and bit index
    DrawText("Byte Index:", windowBounds.x + 420, windowBounds.y + 35, 12, BLACK);
    Rectangle byteBounds = {windowBounds.x + 500, windowBounds.y + 35, 60, 20};
    DrawRectangleRec(byteBounds, WHITE);
    DrawRectangleLines(byteBounds.x, byteBounds.y, byteBounds.width, byteBounds.height, BLACK);
    DrawText(std::to_string(byteIndex).c_str(), byteBounds.x + 5, byteBounds.y + 3, 10, BLACK);
    
    DrawText("Bit Index:", windowBounds.x + 580, windowBounds.y + 35, 12, BLACK);
    Rectangle bitBounds = {windowBounds.x + 650, windowBounds.y + 35, 60, 20};
    DrawRectangleRec(bitBounds, WHITE);
    DrawRectangleLines(bitBounds.x, bitBounds.y, bitBounds.width, bitBounds.height, BLACK);
    DrawText(std::to_string(bitIndex).c_str(), bitBounds.x + 5, bitBounds.y + 3, 10, BLACK);
    
    // Draw find section
    DrawText("Find In:", windowBounds.x + 20, windowBounds.y + 95, 14, BLACK);
    
    DrawText("Bus:", windowBounds.x + 30, windowBounds.y + 120, 12, BLACK);
    Rectangle findBusBounds = {windowBounds.x + 100, windowBounds.y + 120, 60, 20};
    DrawRectangleRec(findBusBounds, LIGHTGRAY);
    DrawRectangleLines(findBusBounds.x, findBusBounds.y, findBusBounds.width, findBusBounds.height, BLACK);
    DrawText(std::to_string(findBus).c_str(), findBusBounds.x + 5, findBusBounds.y + 3, 10, BLACK);
    
    DrawText("Equal:", windowBounds.x + 180, windowBounds.y + 120, 12, BLACK);
    Rectangle equalBounds = {windowBounds.x + 250, windowBounds.y + 120, 80, 20};
    DrawRectangleRec(equalBounds, LIGHTGRAY);
    DrawRectangleLines(equalBounds.x, equalBounds.y, equalBounds.width, equalBounds.height, BLACK);
    const char* equalOptions[] = {"Yes", "No"};
    DrawText(equalOptions[equalSelection], equalBounds.x + 5, equalBounds.y + 3, 10, BLACK);
    
    // Draw min msg count
    DrawText("Min msg count:", windowBounds.x + 350, windowBounds.y + 120, 12, BLACK);
    Rectangle minMsgBounds = {windowBounds.x + 450, windowBounds.y + 120, 60, 20};
    DrawRectangleRec(minMsgBounds, WHITE);
    DrawRectangleLines(minMsgBounds.x, minMsgBounds.y, minMsgBounds.width, minMsgBounds.height, BLACK);
    DrawText(minMsgs.c_str(), minMsgBounds.x + 5, minMsgBounds.y + 3, 10, BLACK);
    
    // Draw search button
    DrawRectangleRec(searchButtonBounds, searchInProgress ? GRAY : GREEN);
    DrawRectangleLines(searchButtonBounds.x, searchButtonBounds.y, searchButtonBounds.width, searchButtonBounds.height, BLACK);
    DrawText(searchInProgress ? "Finding..." : "Find", searchButtonBounds.x + 25, searchButtonBounds.y + 8, 12, WHITE);
    
    // Draw table header
    const char* headers[] = {"address", "byte idx", "bit idx", "mismatches", "total msgs", "% mismatched"};
    float col_width = tableBounds.width / 6.0f;
    
    for (int i = 0; i < 6; i++) {
        Rectangle header_rect = {tableBounds.x + i * col_width, tableBounds.y, col_width, 20};
        DrawRectangleRec(header_rect, Color{200, 200, 200, 255});
        DrawRectangleLines(header_rect.x, header_rect.y, header_rect.width, header_rect.height, BLACK);
        DrawText(headers[i], header_rect.x + 5, header_rect.y + 3, 10, BLACK);
    }
    
    // Draw table rows
    drawTable(tableBounds);
}

void FindSimilarBitsDlg::drawTable(const Rectangle& bounds) {
    float row_height = 20;
    int start_row = static_cast<int>(scroll_offset_ / row_height);
    int end_row = std::min(static_cast<int>(results.size()), 
                          start_row + static_cast<int>(bounds.height / row_height) + 1);
    
    float y_pos = bounds.y + 20 - fmod(scroll_offset_, row_height);
    
    for (int i = start_row; i < end_row && i < static_cast<int>(results.size()); i++) {
        // Highlight selected row
        if (i == selectedRow) {
            DrawRectangle(bounds.x, y_pos, bounds.width, row_height, Color{180, 180, 255, 255});
        }
        
        // Draw row separators
        DrawRectangleLines(bounds.x, y_pos, bounds.width, row_height, LIGHTGRAY);
        
        // Draw data in columns
        float col_width = bounds.width / 6.0f;
        
        // Column 0: address (hex)
        std::string addr_str = "0x" + std::to_string(results[i].address);
        DrawText(addr_str.c_str(), bounds.x + 5, y_pos + 2, 10, DARKGRAY);
        
        // Column 1: byte index
        DrawText(std::to_string(results[i].byte_idx).c_str(), bounds.x + col_width + 5, y_pos + 2, 10, DARKGRAY);
        
        // Column 2: bit index
        DrawText(std::to_string(results[i].bit_idx).c_str(), bounds.x + 2 * col_width + 5, y_pos + 2, 10, DARKGRAY);
        
        // Column 3: mismatches
        DrawText(std::to_string(results[i].mismatches).c_str(), bounds.x + 3 * col_width + 5, y_pos + 2, 10, DARKGRAY);
        
        // Column 4: total messages
        DrawText(std::to_string(results[i].total).c_str(), bounds.x + 4 * col_width + 5, y_pos + 2, 10, DARKGRAY);
        
        // Column 5: percentage
        char perc_str[16];
        snprintf(perc_str, sizeof(perc_str), "%.2f", results[i].perc);
        DrawText(perc_str, bounds.x + 5 * col_width + 5, y_pos + 2, 10, DARKGRAY);
        
        y_pos += row_height;
    }
    
    // Draw scroll indicator
    if (results.size() > 0) {
        float max_scroll = std::max(0.0f, static_cast<float>(results.size()) * row_height - (bounds.height - 20));
        if (max_scroll > 0) {
            float scroll_ratio = scroll_offset_ / max_scroll;
            float scrollbar_height = (bounds.height - 20) * ((bounds.height - 20) / row_height) / results.size();
            if (scrollbar_height < 10) scrollbar_height = 10;
            if (scrollbar_height > bounds.height - 20) scrollbar_height = bounds.height - 20;
            
            float scrollbar_y = bounds.y + 20 + (bounds.height - 20 - scrollbar_height) * scroll_ratio;
            DrawRectangle(bounds.x + bounds.width - 15, scrollbar_y, 10, scrollbar_height, GRAY);
        }
    }
}

void FindSimilarBitsDlg::handleInput() {
    // Already handled in update() method
}

void FindSimilarBitsDlg::find() {
    searchInProgress = true;
    
    // Prepare search parameters
    BitSearchParams params;
    params.src_bus = static_cast<uint8_t>(srcBus);
    params.selected_address = selectedAddress;  // This would come from the selected message
    params.byte_idx = byteIndex;
    params.bit_idx = bitIndex;
    params.find_bus = static_cast<uint8_t>(findBus);
    params.equal = (equalSelection == 0);  // 0 = Yes (equal), 1 = No (not equal)
    params.min_msgs_cnt = minMsgs.empty() ? 100 : std::stoi(minMsgs);
    
    // Perform the search
    results = calcBits(params);
    
    searchInProgress = false;
}

std::vector<MismatchedStruct> FindSimilarBitsDlg::calcBits(const BitSearchParams& params) {
    std::vector<MismatchedStruct> result;
    
    // This is a simplified simulation - in a real implementation, this would:
    // 1. Go through CAN events from both buses
    // 2. Compare the specified bit from source message with corresponding bits in find bus
    // 3. Calculate mismatches based on the equality parameter
    
    // For now, generating some sample results
    for (int i = 0; i < 10; i++) {
        MismatchedStruct ms;
        ms.address = 0x123 + i * 0x10;  // Example addresses
        ms.byte_idx = params.byte_idx;
        ms.bit_idx = params.bit_idx;
        ms.mismatches = i * 5;  // Example mismatch values
        ms.total = params.min_msgs_cnt + i * 10;  // Example total messages
        ms.perc = (ms.mismatches / static_cast<float>(ms.total)) * 100.0f;
        result.push_back(ms);
    }
    
    // Sort by percentage (ascending)
    std::sort(result.begin(), result.end(), [](const MismatchedStruct& a, const MismatchedStruct& b) {
        return a.perc < b.perc;
    });
    
    return result;
}

int FindSimilarBitsDlg::getRowAtPosition(float y, const Rectangle& bounds) const {
    float row_height = 20;
    float relative_y = y - bounds.y - 20;  // Offset for header
    int row = static_cast<int>((relative_y + scroll_offset_) / row_height);
    
    if (row >= 0 && row < static_cast<int>(results.size())) {
        return row;
    }
    return -1;
}

