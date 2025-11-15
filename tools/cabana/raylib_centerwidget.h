#pragma once

#include <memory>
#include "raylib.h"

// Implementation for CenterWidget in Raylib-based UI
class CenterWidget {
public:
    CenterWidget(void* parent = nullptr);
    ~CenterWidget();

    void update();
    void render(const Rectangle& bounds);
    void clear();
    void setMessage(const void* message);  // MessageId would need to be passed differently
    void setVisible(bool visible) { is_visible_ = visible; }
    bool isVisible() const { return is_visible_; }

private:
    void* parent_;
    bool is_visible_ = true;
};