#pragma once

#include <vector>
#include <string>
#include <functional>

#include "raylib.h"
#include "tools/cabana/dbc/dbcmanager.h"

// Data structure for mismatched bits
struct MismatchedStruct {
    uint32_t address;
    uint32_t byte_idx;
    uint32_t bit_idx;
    uint32_t mismatches;
    uint32_t total;
    float perc;
};

// Data structure for bit search parameters
struct BitSearchParams {
    uint8_t src_bus = 0;
    uint32_t selected_address = 0;
    int byte_idx = 0;
    int bit_idx = 0;
    uint8_t find_bus = 0;
    bool equal = true;
    int min_msgs_cnt = 100;
};

// Raylib-based FindSimilarBitsDlg
class FindSimilarBitsDlg {
public:
    FindSimilarBitsDlg(void* parent = nullptr);
    ~FindSimilarBitsDlg();
    
    void show();
    void hide();
    bool isVisible() const { return visible; }
    void update();
    void render(const Rectangle& bounds);
    void handleInput();

    // Callback for when a message should be opened
    std::function<void(const MessageId&)> onOpenMessage;

private:
    void find();
    std::vector<MismatchedStruct> calcBits(const BitSearchParams& params);
    void drawTable(const Rectangle& bounds);
    int getRowAtPosition(float y, const Rectangle& bounds) const;

    // UI state
    bool visible = false;
    Rectangle windowBounds;
    std::vector<MismatchedStruct> results;
    
    // Input parameters
    int srcBus = 0;
    int findBus = 0;
    uint32_t selectedAddress = 0;
    int byteIndex = 0;
    int bitIndex = 0;
    int equalSelection = 0; // 0=Yes, 1=No
    std::string minMsgs = "100";
    
    // UI elements bounds
    Rectangle searchButtonBounds;
    Rectangle tableBounds;
    
    // UI state
    int selectedRow = -1;
    bool searchInProgress = false;
    std::vector<int> availableBuses; // Available bus options
    std::vector<std::pair<std::string, uint32_t>> availableMessages; // Available message options
    float scroll_offset_ = 0.0f;
    double lastClickTime = 0.0;
};

extern FindSimilarBitsDlg* findSimilarBitsDlg;