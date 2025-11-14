#pragma once

#include <memory>

// Placeholder for CenterWidget - in raylib implementation this may not be needed
// or may be implemented differently than the QT version
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