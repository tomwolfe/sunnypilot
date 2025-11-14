#pragma once

#include <set>
#include <string>

#include "raylib.h"
#include "tools/cabana/raylib_binaryview.h"
#include "tools/cabana/raylib_chartswidget.h"
#include "tools/cabana/raylib_signalview.h"
#include "tools/cabana/raylib_historylog.h"
#include "tools/cabana/utils/util.h"

// Data structures for message details
struct MessageDetails {
    MessageId msg_id;
    std::string name;
    int address;
    int size;
    std::string comment;
    std::string node;
};

// Raylib-based DetailWidget
class DetailWidget {
public:
    DetailWidget(void* charts_widget, void* parent = nullptr);
    ~DetailWidget();
    
    void update();
    void render(const Rectangle& bounds);
    void handleInput();
    
    void setMessage(const MessageId &message_id);
    void refresh();
    
    // Accessor methods
    const MessageDetails& getMessageDetails() const { return msg_details_; }
    void setChartsWidget(void* charts) { charts_ = charts; }
    
    // Signals equivalent (using callbacks)
    std::function<void()> onMessageEdited;
    std::function<void()> onMessageRemoved;

private:
    void createToolBar();
    void updateState();
    
    MessageDetails msg_details_;
    MessageId msg_id_;
    
    // UI Components
    std::unique_ptr<BinaryView> binary_view_;
    std::unique_ptr<SignalView> signal_view_;
    std::unique_ptr<LogsWidget> history_log_;
    
    // References
    void* charts_ = nullptr;
    void* parent_ = nullptr;
    
    // UI state
    Rectangle bounds_;
    bool is_visible_ = true;
    
    // Layout state
    bool show_binary_view_ = true;
    bool show_signal_view_ = true;
    bool show_history_log_ = true;
    int active_tab_ = 0;  // 0=Binary, 1=Signals, 2=History
};

// Raylib-based CenterWidget
class CenterWidget {
public:
    CenterWidget(void* parent = nullptr);
    ~CenterWidget();
    
    void update();
    void render(const Rectangle& bounds);
    void handleInput();
    
    void setMessage(const MessageId &msg_id);
    void clear();
    
    // Accessor methods
    DetailWidget* getDetailWidget() { return detail_widget_.get(); }
    
private:
    std::unique_ptr<DetailWidget> detail_widget_;
    std::unique_ptr<void> welcome_widget_; // Placeholder for welcome interface
    
    Rectangle bounds_;
    bool is_visible_ = true;
    bool show_welcome_ = true;
    MessageId current_msg_id_;
};