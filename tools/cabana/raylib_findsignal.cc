#include "raylib_findsignal.h"
#include <sstream>
#include <algorithm>
#include <cctype>

// Global instance
FindSignalDlg* findSignalDlg = nullptr;

FindSignalModel::FindSignalModel() = default;

void FindSignalModel::updateMsbLsb(cabana::Signal& sig) {
    // Update MSB/LSB based on endianness
    if (sig.is_little_endian) {
        // For little endian
        sig.start_bit = (sig.start_bit / 8) * 8 + (7 - (sig.start_bit % 8));
    }
}

void FindSignalModel::search(std::function<bool(double)> cmp) {
    // This is a simplified implementation
    // In a real implementation, this would search through CAN events
    // For now, we'll keep the current filtered_signals as is
    
    // In a real implementation:
    // 1. Get previous signals from histories or initial_signals
    // 2. Filter based on the comparison function
    // 3. Add results to filtered_signals
    // 4. Push filtered_signals to histories
    
    // For now, just store a placeholder
    histories.push_back(filtered_signals);
}

void FindSignalModel::undo() {
    if (!histories.empty()) {
        histories.pop_back();
        filtered_signals.clear();
        if (!histories.empty()) {
            filtered_signals = histories.back();
        }
    }
}

void FindSignalModel::reset() {
    histories.clear();
    filtered_signals.clear();
    initial_signals.clear();
}

// FindSignalDlg implementation
FindSignalDlg::FindSignalDlg(void* parent) {
    windowBounds = {200, 100, 800, 650};  // x, y, width, height
    visible = false;
    
    // Initialize UI element bounds
    searchButtonBounds = {windowBounds.x + 650, windowBounds.y + 450, 100, 30};
    resetButtonBounds = {windowBounds.x + 500, windowBounds.y + 450, 100, 30};
    undoButtonBounds = {windowBounds.x + 350, windowBounds.y + 450, 120, 30};
    tableViewBounds = {windowBounds.x + 20, windowBounds.y + 490, 760, 140};
    
    // Initialize default values
    factor = "1.0";
    offset = "0.0";
    first_time = "0";
    last_time = "MAX";
    minSize = 8;
    maxSize = 8;
    
    // Initialize compare selection (0 = "=")
    compareSelection = 0;
}

FindSignalDlg::~FindSignalDlg() = default;

void FindSignalDlg::show() {
    visible = true;
}

void FindSignalDlg::hide() {
    visible = false;
}

void FindSignalDlg::update() {
    if (!visible) return;

    Vector2 mousePos = GetMousePosition();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        // Check search button
        if (CheckCollisionPointRec(mousePos, searchButtonBounds)) {
            search();
        }
        // Check reset button
        else if (CheckCollisionPointRec(mousePos, resetButtonBounds)) {
            model.reset();
            modelReset();
        }
        // Check undo button
        else if (CheckCollisionPointRec(mousePos, undoButtonBounds)) {
            model.undo();
            modelReset();
        }
        // Check table view for selection
        else if (CheckCollisionPointRec(mousePos, tableViewBounds)) {
            int row = getRowAtPosition(mousePos.y, tableViewBounds);
            if (row >= 0 && row < static_cast<int>(model.filtered_signals.size())) {
                selectedRow = row;
                
                // Double-click simulation (in real app this would be two quick clicks)
                if (GetTime() - lastClickTime < 0.5) {  // Double click threshold
                    if (onOpenMessage && selectedRow >= 0 && selectedRow < model.filtered_signals.size()) {
                        onOpenMessage(model.filtered_signals[selectedRow].id);
                    }
                }
                lastClickTime = GetTime();
            }
        }
    }
    
    // Handle text input for relevant fields
    // For simplicity, we'll use a basic approach - in a real implementation 
    // you'd have a more sophisticated input handling system
}

void FindSignalDlg::render(const Rectangle& parentBounds) {
    if (!visible) return;
    
    // Draw modal background
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(GRAY, 0.5f));
    
    // Draw window background
    DrawRectangleRec(windowBounds, Color{245, 245, 245, 255}); // Light gray
    DrawRectangleLines(windowBounds.x, windowBounds.y, windowBounds.width, windowBounds.height, BLACK);
    
    // Draw window title
    DrawText("Find Signal", windowBounds.x + 10, windowBounds.y + 5, 20, BLACK);
    
    // Draw message group
    Rectangle msgGroupBounds = {windowBounds.x + 20, windowBounds.y + 30, 350, 120};
    DrawRectangleRec(msgGroupBounds, Color{230, 230, 230, 255});
    DrawRectangleLines(msgGroupBounds.x, msgGroupBounds.y, msgGroupBounds.width, msgGroupBounds.height, BLACK);
    DrawText("Messages", msgGroupBounds.x + 5, msgGroupBounds.y + 5, 14, BLACK);
    
    // Draw bus input
    DrawText("Bus:", msgGroupBounds.x + 10, msgGroupBounds.y + 30, 12, BLACK);
    Rectangle busInputBounds = {msgGroupBounds.x + 50, msgGroupBounds.y + 25, 150, 20};
    DrawRectangleRec(busInputBounds, WHITE);
    DrawRectangleLines(busInputBounds.x, busInputBounds.y, busInputBounds.width, busInputBounds.height, BLACK);
    DrawText(bus.empty() ? "comma-seperated values. Leave blank for all" : bus.c_str(), 
             busInputBounds.x + 5, busInputBounds.y + 2, 10, bus.empty() ? GRAY : BLACK);
    
    // Draw address input
    DrawText("Address:", msgGroupBounds.x + 10, msgGroupBounds.y + 60, 12, BLACK);
    Rectangle addrInputBounds = {msgGroupBounds.x + 70, msgGroupBounds.y + 55, 150, 20};
    DrawRectangleRec(addrInputBounds, WHITE);
    DrawRectangleLines(addrInputBounds.x, addrInputBounds.y, addrInputBounds.width, addrInputBounds.height, BLACK);
    DrawText(address.empty() ? "comma-seperated hex values. Leave blank for all" : address.c_str(), 
             addrInputBounds.x + 5, addrInputBounds.y + 2, 10, address.empty() ? GRAY : BLACK);
    
    // Draw time inputs
    DrawText("Time:", msgGroupBounds.x + 10, msgGroupBounds.y + 90, 12, BLACK);
    Rectangle time1Bounds = {msgGroupBounds.x + 50, msgGroupBounds.y + 85, 60, 20};
    DrawRectangleRec(time1Bounds, WHITE);
    DrawRectangleLines(time1Bounds.x, time1Bounds.y, time1Bounds.width, time1Bounds.height, BLACK);
    DrawText(first_time.c_str(), time1Bounds.x + 5, time1Bounds.y + 2, 10, BLACK);
    
    DrawText("-", msgGroupBounds.x + 115, msgGroupBounds.y + 87, 12, BLACK);
    
    Rectangle time2Bounds = {msgGroupBounds.x + 130, msgGroupBounds.y + 85, 60, 20};
    DrawRectangleRec(time2Bounds, WHITE);
    DrawRectangleLines(time2Bounds.x, time2Bounds.y, time2Bounds.width, time2Bounds.height, BLACK);
    DrawText(last_time.c_str(), time2Bounds.x + 5, time2Bounds.y + 2, 10, BLACK);
    
    DrawText("seconds", msgGroupBounds.x + 195, msgGroupBounds.y + 87, 10, BLACK);
    
    // Draw properties group
    Rectangle propGroupBounds = {windowBounds.x + 380, windowBounds.y + 30, 400, 140};
    DrawRectangleRec(propGroupBounds, Color{230, 230, 230, 255});
    DrawRectangleLines(propGroupBounds.x, propGroupBounds.y, propGroupBounds.width, propGroupBounds.height, BLACK);
    DrawText("Signal", propGroupBounds.x + 5, propGroupBounds.y + 5, 14, BLACK);
    
    // Draw size controls
    DrawText("Size:", propGroupBounds.x + 10, propGroupBounds.y + 30, 12, BLACK);
    Rectangle minSizeBounds = {propGroupBounds.x + 50, propGroupBounds.y + 25, 50, 20};
    DrawRectangleRec(minSizeBounds, WHITE);
    DrawRectangleLines(minSizeBounds.x, minSizeBounds.y, minSizeBounds.width, minSizeBounds.height, BLACK);
    DrawText(std::to_string(minSize).c_str(), minSizeBounds.x + 5, minSizeBounds.y + 2, 10, BLACK);
    
    DrawText("-", propGroupBounds.x + 105, propGroupBounds.y + 27, 12, BLACK);
    
    Rectangle maxSizeBounds = {propGroupBounds.x + 120, propGroupBounds.y + 25, 50, 20};
    DrawRectangleRec(maxSizeBounds, WHITE);
    DrawRectangleLines(maxSizeBounds.x, maxSizeBounds.y, maxSizeBounds.width, maxSizeBounds.height, BLACK);
    DrawText(std::to_string(maxSize).c_str(), maxSizeBounds.x + 5, maxSizeBounds.y + 2, 10, BLACK);
    
    // Draw checkboxes
    Rectangle endianBounds = {propGroupBounds.x + 180, propGroupBounds.y + 25, 15, 15};
    DrawRectangleRec(endianBounds, WHITE);
    DrawRectangleLines(endianBounds.x, endianBounds.y, endianBounds.width, endianBounds.height, BLACK);
    if (littleEndian) {
        DrawRectangle(endianBounds.x + 2, endianBounds.y + 2, endianBounds.width - 4, endianBounds.height - 4, BLACK);
    }
    DrawText("Little endian", endianBounds.x + 20, endianBounds.y, 10, BLACK);
    
    Rectangle signedBounds = {propGroupBounds.x + 180, propGroupBounds.y + 45, 15, 15};
    DrawRectangleRec(signedBounds, WHITE);
    DrawRectangleLines(signedBounds.x, signedBounds.y, signedBounds.width, signedBounds.height, BLACK);
    if (isSigned) {
        DrawRectangle(signedBounds.x + 2, signedBounds.y + 2, signedBounds.width - 4, signedBounds.height - 4, BLACK);
    }
    DrawText("Signed", signedBounds.x + 20, signedBounds.y + 20, 10, BLACK);
    
    // Draw factor and offset
    DrawText("Factor:", propGroupBounds.x + 10, propGroupBounds.y + 70, 12, BLACK);
    Rectangle factorBounds = {propGroupBounds.x + 50, propGroupBounds.y + 65, 80, 20};
    DrawRectangleRec(factorBounds, WHITE);
    DrawRectangleLines(factorBounds.x, factorBounds.y, factorBounds.width, factorBounds.height, BLACK);
    DrawText(factor.c_str(), factorBounds.x + 5, factorBounds.y + 2, 10, BLACK);
    
    DrawText("Offset:", propGroupBounds.x + 140, propGroupBounds.y + 70, 12, BLACK);
    Rectangle offsetBounds = {propGroupBounds.x + 180, propGroupBounds.y + 65, 80, 20};
    DrawRectangleRec(offsetBounds, WHITE);
    DrawRectangleLines(offsetBounds.x, offsetBounds.y, offsetBounds.width, offsetBounds.height, BLACK);
    DrawText(offset.c_str(), offsetBounds.x + 5, offsetBounds.y + 2, 10, BLACK);
    
    // Draw find group (controls)
    Rectangle findGroupBounds = {windowBounds.x + 20, windowBounds.y + 160, 760, 100};
    DrawRectangleRec(findGroupBounds, Color{230, 230, 230, 255});
    DrawRectangleLines(findGroupBounds.x, findGroupBounds.y, findGroupBounds.width, findGroupBounds.height, BLACK);
    DrawText("Find signal", findGroupBounds.x + 5, findGroupBounds.y + 5, 14, BLACK);
    
    // Draw value comparison controls
    DrawText("Value:", findGroupBounds.x + 10, findGroupBounds.y + 30, 12, BLACK);
    
    Rectangle compareCbBounds = {findGroupBounds.x + 50, findGroupBounds.y + 25, 60, 20};
    DrawRectangleRec(compareCbBounds, LIGHTGRAY);
    DrawRectangleLines(compareCbBounds.x, compareCbBounds.y, compareCbBounds.width, compareCbBounds.height, BLACK);
    const char* compareOptions[] = {"=", ">", ">=", "!=", "<", "<=", "between"};
    DrawText(compareOptions[compareSelection], compareCbBounds.x + 5, compareCbBounds.y + 3, 10, BLACK);
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mousePos, compareCbBounds)) {
        compareSelection = (compareSelection + 1) % 7;
    }
    
    Rectangle value1Bounds = {findGroupBounds.x + 120, findGroupBounds.y + 25, 80, 20};
    DrawRectangleRec(value1Bounds, WHITE);
    DrawRectangleLines(value1Bounds.x, value1Bounds.y, value1Bounds.width, value1Bounds.height, BLACK);
    DrawText(value1.c_str(), value1Bounds.x + 5, value1Bounds.y + 2, 10, BLACK);
    
    // Draw "to" label and value2 if needed
    if (compareSelection == 6) { // "between" option
        DrawText("-", findGroupBounds.x + 210, findGroupBounds.y + 27, 12, BLACK);
        Rectangle value2Bounds = {findGroupBounds.x + 230, findGroupBounds.y + 25, 80, 20};
        DrawRectangleRec(value2Bounds, WHITE);
        DrawRectangleLines(value2Bounds.x, value2Bounds.y, value2Bounds.width, value2Bounds.height, BLACK);
        DrawText(value2.c_str(), value2Bounds.x + 5, value2Bounds.y + 2, 10, BLACK);
    }
    
    // Draw buttons
    DrawRectangleRec(undoButtonBounds, ORANGE);
    DrawRectangleLines(undoButtonBounds.x, undoButtonBounds.y, undoButtonBounds.width, undoButtonBounds.height, BLACK);
    DrawText("Undo prev find", undoButtonBounds.x + 10, undoButtonBounds.y + 8, 10, WHITE);
    
    DrawRectangleRec(searchButtonBounds, GREEN);
    DrawRectangleLines(searchButtonBounds.x, searchButtonBounds.y, searchButtonBounds.width, searchButtonBounds.height, BLACK);
    DrawText(searchInProgress ? "Finding..." : "Find", searchButtonBounds.x + 20, searchButtonBounds.y + 8, 12, WHITE);
    
    DrawRectangleRec(resetButtonBounds, RED);
    DrawRectangleLines(resetButtonBounds.x, resetButtonBounds.y, resetButtonBounds.width, resetButtonBounds.height, BLACK);
    DrawText("Reset", resetButtonBounds.x + 35, resetButtonBounds.y + 8, 12, WHITE);
    
    // Draw table view
    DrawRectangleRec(tableViewBounds, WHITE);
    DrawRectangleLines(tableViewBounds.x, tableViewBounds.y, tableViewBounds.width, tableViewBounds.height, BLACK);
    
    // Draw table header
    const char* headers[] = {"Id", "Start Bit, size", "(time, value)"};
    float col_width = tableViewBounds.width / 3.0f;
    
    for (int i = 0; i < 3; i++) {
        Rectangle header_rect = {tableViewBounds.x + i * col_width, tableViewBounds.y, col_width, 20};
        DrawRectangleRec(header_rect, Color{200, 200, 200, 255});
        DrawRectangleLines(header_rect.x, header_rect.y, header_rect.width, header_rect.height, BLACK);
        DrawText(headers[i], header_rect.x + 5, header_rect.y + 3, 10, BLACK);
    }
    
    // Draw table rows
    drawTable(tableViewBounds);
    
    // Draw status text
    if (!statusText.empty()) {
        DrawText(statusText.c_str(), windowBounds.x + 20, windowBounds.y + 470, 10, BLACK);
    }
}

void FindSignalDlg::drawTable(const Rectangle& bounds) {
    float row_height = 20;
    int start_row = std::max(0, static_cast<int>(scroll_offset_));
    int end_row = std::min(static_cast<int>(model.filtered_signals.size()), 
                          start_row + static_cast<int>((bounds.height - 20) / row_height));
    
    float y_pos = bounds.y + 20;
    for (int i = start_row; i < end_row; i++) {
        if (i >= model.filtered_signals.size()) break;
        
        const auto& signal = model.filtered_signals[i];
        
        // Highlight selected row
        if (i == selectedRow) {
            DrawRectangle(bounds.x, y_pos, bounds.width, row_height, Color{180, 180, 255, 255});
        }
        
        // Draw row separators
        DrawRectangleLines(bounds.x, y_pos, bounds.width, row_height, LIGHTGRAY);
        
        // Draw data in columns
        float col_width = bounds.width / 3.0f;
        
        // Column 1: Id
        std::string id_str = "0x" + std::to_string(signal.id.address);
        DrawText(id_str.c_str(), bounds.x + 5, y_pos + 2, 10, DARKGRAY);
        
        // Column 2: Start bit, size
        std::string size_str = std::to_string(signal.sig.start_bit) + ", " + std::to_string(signal.sig.size);
        DrawText(size_str.c_str(), bounds.x + col_width + 5, y_pos + 2, 10, DARKGRAY);
        
        // Column 3: Values
        std::string values_str;
        for (size_t j = 0; j < std::min(signal.values.size(), size_t(3)); j++) { // Only show first few values
            if (j > 0) values_str += " ";
            values_str += signal.values[j];
        }
        if (signal.values.size() > 3) values_str += "...";
        DrawText(values_str.c_str(), bounds.x + 2 * col_width + 5, y_pos + 2, 10, DARKGRAY);
        
        y_pos += row_height;
    }
    
    // Draw scroll indicator
    if (model.filtered_signals.size() > 0) {
        float max_scroll = static_cast<float>(model.filtered_signals.size()) - (bounds.height - 20) / row_height;
        if (max_scroll > 0) {
            float scroll_ratio = scroll_offset_ / max_scroll;
            float scrollbar_height = (bounds.height - 20) * ((bounds.height - 20) / row_height) / model.filtered_signals.size();
            if (scrollbar_height < 10) scrollbar_height = 10;
            if (scrollbar_height > bounds.height - 20) scrollbar_height = bounds.height - 20;
            
            float scrollbar_y = bounds.y + 20 + (bounds.height - 20 - scrollbar_height) * scroll_ratio;
            DrawRectangle(bounds.x + bounds.width - 15, scrollbar_y, 10, scrollbar_height, GRAY);
        }
    }
}

void FindSignalDlg::handleInput() {
    // Already handled in update() method
}

void FindSignalDlg::search() {
    if (model.histories.empty()) {
        setInitialSignals();
    }
    
    // Parse values
    double v1 = value1.empty() ? 0.0 : std::stod(value1);
    double v2 = value2.empty() ? 0.0 : std::stod(value2);
    
    std::function<bool(double)> cmp = nullptr;
    switch (compareSelection) {
        case 0: cmp = [v1](double v) { return v == v1; }; break;
        case 1: cmp = [v1](double v) { return v > v1; }; break;
        case 2: cmp = [v1](double v) { return v >= v1; }; break;
        case 3: cmp = [v1](double v) { return v != v1; }; break;
        case 4: cmp = [v1](double v) { return v < v1; }; break;
        case 5: cmp = [v1](double v) { return v <= v1; }; break;
        case 6: cmp = [v1, v2](double v) { return v >= v1 && v <= v2; }; break;
    }
    
    if (cmp) {
        searchInProgress = true;
        model.search(cmp);
        searchInProgress = false;
    }
    
    modelReset();
}

void FindSignalDlg::modelReset() {
    bool isEmptyHistory = model.histories.empty();
    
    // Update status text
    statusText = std::to_string(model.filtered_signals.size()) + " matches. Double click on an item to open message";
}

void FindSignalDlg::setInitialSignals() {
    // This is a simplified version - in a real implementation, this would parse
    // bus and address information and build the initial signals list from CAN messages
    
    // For now, adding some placeholder data
    model.initial_signals.clear();
    
    // In a real implementation, this would:
    // 1. Parse bus_edit and address_edit
    // 2. Iterate through CAN messages
    // 3. Generate possible signals based on minSize/maxSize constraints
    // 4. Store them in model.initial_signals
}

int FindSignalDlg::getRowAtPosition(float y, const Rectangle& bounds) const {
    float row_height = 20;
    float relative_y = y - bounds.y - 20;  // Offset for header
    int row = static_cast<int>(relative_y / row_height);
    
    int start_row = static_cast<int>(scroll_offset_);
    int actual_row = start_row + row;
    
    if (actual_row >= 0 && actual_row < static_cast<int>(model.filtered_signals.size())) {
        return actual_row;
    }
    return -1;
}

