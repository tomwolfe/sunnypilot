#pragma once

#include <vector>
#include <string>
#include <functional>

#include "raylib.h"
#include "tools/cabana/dbc/dbcmanager.h"

// Data structure for search signal
struct SearchSignal {
    MessageId id;
    uint64_t mono_time = 0;
    cabana::Signal sig = {};
    double value = 0.0;
    std::vector<std::string> values;
};

// Raylib-based FindSignalModel
class FindSignalModel {
public:
    FindSignalModel();
    void search(std::function<bool(double)> cmp);
    void reset();
    void undo();

    std::vector<SearchSignal> filtered_signals;
    std::vector<SearchSignal> initial_signals;
    std::vector<std::vector<SearchSignal>> histories;
    uint64_t last_time = UINT64_MAX;  // Use UINT64_MAX instead of std::numeric_limits
    
private:
    void updateMsbLsb(cabana::Signal& sig);  // Helper function
};

// Raylib-based FindSignalDlg
class FindSignalDlg {
public:
    FindSignalDlg(void* parent = nullptr);
    ~FindSignalDlg();
    
    void show();
    void hide();
    bool isVisible() const { return visible; }
    void update();
    void render(const Rectangle& bounds);
    void handleInput();

    // Data access
    const std::vector<SearchSignal>& getFilteredSignals() const { return model.filtered_signals; }

    // Callback for when a message should be opened
    std::function<void(const MessageId&)> onOpenMessage;

private:
    void search();
    void modelReset();
    void setInitialSignals();
    void drawTable(const Rectangle& bounds);
    int getRowAtPosition(float y, const Rectangle& bounds) const;
    int selectedRow = -1;

    // UI state
    bool visible = false;
    Rectangle windowBounds;
    
    // Input fields
    std::string value1, value2, factor, offset;
    std::string bus, address, first_time, last_time;
    int compareSelection = 0; // 0="=", 1=">", 2=">=", 3="!=", 4="<", 5="<=", 6="between"
    int minSize = 8, maxSize = 8;
    bool littleEndian = true;
    bool isSigned = false;
    
    // UI elements bounds
    Rectangle searchButtonBounds;
    Rectangle resetButtonBounds;
    Rectangle undoButtonBounds;
    Rectangle tableViewBounds;
    
    // Model
    FindSignalModel model;
    
    // UI state
    bool searchInProgress = false;
    std::string statusText;
    double lastClickTime = 0.0;
    float scroll_offset_ = 0.0f;
};

extern FindSignalDlg* findSignalDlg;