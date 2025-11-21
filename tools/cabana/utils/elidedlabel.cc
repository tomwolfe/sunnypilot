#include "elidedlabel.h"
#include <algorithm>
#include <string>

ElidedLabel::ElidedLabel(void* parent) {
    // Initialize with empty text
    text_ = "";
    elided_text_ = "";
}

ElidedLabel::ElidedLabel(const std::string &text, void* parent) {
    text_ = text;
    elided_text_ = text;
}

void ElidedLabel::setText(const std::string &text) {
    text_ = text;
    // For now, just store the text - we'll handle eliding in render
    elided_text_ = text;
}

void ElidedLabel::render(const Rectangle &bounds, int fontSize, Color color) {
    bounds_ = bounds;  // Store for click detection
    
    // Simple eliding implementation
    elided_text_ = text_;
    if (text_.length() > 0) {
        // Measure text width against available space
        float text_width = MeasureText(text_.c_str(), fontSize);
        if (text_width > bounds.width) {
            // Need to elide - try shorter and shorter strings until it fits
            size_t len = text_.length();
            while (len > 3) {
                std::string candidate = text_.substr(0, len - 3) + "...";
                if (MeasureText(candidate.c_str(), fontSize) <= bounds.width) {
                    elided_text_ = candidate;
                    break;
                }
                --len;
            }
        }
    }
    
    // Draw the text
    DrawText(elided_text_.c_str(), bounds.x, bounds.y, fontSize, color);
}

bool ElidedLabel::wasClicked() {
    Vector2 mouse_pos = GetMousePosition();
    bool current_click = IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && 
                         CheckCollisionPointRec(mouse_pos, bounds_);
    
    if (current_click && !last_click_state_) {
        if (onClick) {
            onClick();
        }
        return true;
    }
    
    last_click_state_ = current_click;
    return false;
}