#pragma once

#include <memory>

// Implementation for CenterWidget in Raylib-based UI
class CenterWidget {
public:
    CenterWidget(void* parent = nullptr);
    ~CenterWidget();
    
    void clear();
    void setMessage(const void* message);  // MessageId would need to be passed differently

private:
    void* parent_;
    bool is_visible_ = true;
};