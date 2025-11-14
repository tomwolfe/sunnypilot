#pragma once

#include <tuple>
#include <vector>
#include <array>
#include <optional>

#include "raylib.h"
#include "tools/cabana/dbc/dbcmanager.h"
#include "tools/cabana/streams/abstractstream.h"

// Data structure for binary view item
struct BinaryViewItem {
    Color bg_color = {102, 86, 169, 255};
    bool is_msb = false;
    bool is_lsb = false;
    uint8_t val = 0;
    // For simplicity, using a vector instead of QList
    std::vector<const cabana::Signal*> sigs;
    bool valid = false;
};

// Raylib-based BinaryView
class BinaryView {
public:
    BinaryView(void* parent = nullptr);
    ~BinaryView();
    
    void update();
    void render(const Rectangle& bounds);
    void handleInput();
    
    void setMessage(const MessageId &message_id);
    void highlight(const cabana::Signal *sig);
    std::vector<const cabana::Signal*> getOverlappingSignals() const;
    void updateState();
    
    // Accessor methods
    bool isMessageActive() const { return is_message_active; }
    void setHeatmapLiveMode(bool live) { heatmap_live_mode = live; }
    
    // Signals equivalent - using callback functions
    std::function<void(const cabana::Signal*)> onSignalClicked;
    std::function<void(const cabana::Signal*)> onSignalHovered;
    std::function<void(const MessageId&, const cabana::Signal*, bool, bool)> onShowChart;

private:
    void drawBinaryData(const Rectangle& bounds);
    void handleMouseInput(const Vector2& mouse_pos, const Rectangle& bounds);
    int getColumnAtPosition(float x, const Rectangle& bounds);
    int getRowAtPosition(float y, const Rectangle& bounds);
    
    std::vector<BinaryViewItem> items_;
    std::vector<std::array<uint32_t, 8>> bit_flip_counts_;
    
    // UI State
    MessageId msg_id_;
    bool is_message_active = false;
    bool heatmap_live_mode = true;
    Rectangle bounds_;
    bool is_visible_ = true;
    
    // Interaction state
    Vector2 last_mouse_pos_ = {0, 0};
    const cabana::Signal *hovered_sig = nullptr;
    int selected_row_ = -1;
    int selected_col_ = -1;
    
    // Bit flip tracker
    struct BitFlipTracker {
        std::optional<std::pair<double, double>> time_range;
        std::vector<std::array<uint32_t, 8>> flip_counts;
    } bit_flip_tracker_;
};