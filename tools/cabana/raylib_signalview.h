#pragma once

#include <vector>
#include <string>
#include <memory>

#include "raylib.h"
#include "tools/cabana/dbc/dbcmanager.h"
#include "tools/cabana/chart/chartswidget.h"

// Data structure for signal information
struct SignalInfo {
    std::string name;
    int size;
    std::string receiver_nodes;
    bool is_little_endian;
    bool is_signed;
    double offset;
    double factor;
    std::string type;
    int multiplex_value;
    std::string unit;
    std::string comment;
    double min_value;
    double max_value;
    std::string value_description;
};

// Raylib-based SignalView
class SignalView {
public:
    SignalView(ChartsWidget *charts_widget, void* parent = nullptr);
    ~SignalView();
    
    void update();
    void render(const Rectangle& bounds);
    void handleInput();
    
    void setMessage(const MessageId &id);
    void setFilter(const std::string &filter);
    void selectSignal(const cabana::Signal *sig, bool expand = false);
    void signalHovered(const cabana::Signal *sig);
    void updateState();
    
    // Accessors
    const std::vector<SignalInfo>& getSignals() const { return signals_; }
    int getSelectedSignalIndex() const { return selected_signal_index_; }
    
    // Signals equivalent (using callbacks)
    std::function<void(const cabana::Signal*)> onHighlight;
    std::function<void(const MessageId&, const cabana::Signal*, bool, bool)> onShowChart;

private:
    void updateSignalList();
    void drawSignalRow(int index, float y_pos, const Rectangle& bounds, const SignalInfo& signal);
    void handleScrolling();
    void updateToolBar();
    
    std::vector<SignalInfo> signals_;
    MessageId current_msg_id_;
    std::string filter_text_;
    
    // UI State
    int selected_signal_index_ = -1;
    float scroll_offset_ = 0.0f;
    Rectangle bounds_;
    bool is_visible_ = true;
    
    // Input state
    Vector2 last_mouse_pos_ = {0, 0};
    bool is_dragging_ = false;
    
    // Toolbar elements
    std::string signal_count_text_ = "Signals: 0";
    int sparkline_range_ = 10;  // Default to 10 seconds
    
    // References to other components
    ChartsWidget *charts_widget_;
};