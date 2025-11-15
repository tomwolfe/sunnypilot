#include "tools/cabana/raylib_centerwidget.h"

CenterWidget::CenterWidget(void* parent) : parent_(parent) {
    // Initialize the center widget
}

CenterWidget::~CenterWidget() = default;

void CenterWidget::update() {
    // Update the center widget state
}

void CenterWidget::render(const Rectangle& bounds) {
    // Render the center widget
    // Draw background
    DrawRectangleRec(bounds, Color{245, 245, 245, 255}); // Light gray background
    DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);

    // Draw placeholder text
    const char* text = "Center Panel";
    int text_width = MeasureText(text, 20);
    int x = bounds.x + (bounds.width - text_width) / 2;
    int y = bounds.y + bounds.height / 2 - 10;
    DrawText(text, x, y, 20, DARKGRAY);
}

void CenterWidget::clear() {
    // Clear any displayed content
}

void CenterWidget::setMessage(const void* message) {
    // Handle setting the current message to display
    // In a real implementation, this would update the UI to show message details
}