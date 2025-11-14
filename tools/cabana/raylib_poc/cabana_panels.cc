#include "cabana_panels.h"
#include <raylib.h>
#include <vector>
#include <string>

// Messages Panel Implementation
MessagesPanel::MessagesPanel(float x, float y, float width, float height)
  : CabanaUIElement(x, y, width, height) {
  // Initialize with some sample messages for testing
  messages = {
    "Message 1: 0x100 - Steering Control",
    "Message 2: 0x200 - Engine Data", 
    "Message 3: 0x300 - Brake Information",
    "Message 4: 0x400 - Throttle Position",
    "Message 5: 0x500 - Vehicle Speed",
    "Message 6: 0x600 - Gear Position",
    "Message 7: 0x700 - GPS Data",
    "Message 8: 0x800 - IMU Information"
  };
}

void MessagesPanel::update() {
  // Handle input for message selection
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    Vector2 mousePos = GetMousePosition();
    if (CheckCollisionPointRec(mousePos, bounds)) {
      // Calculate which message was clicked based on scroll position
      float relativeY = mousePos.y - bounds.y;
      int messageIndex = static_cast<int>((relativeY + scrollOffset) / 20); // Approximate message height
      
      if (messageIndex >= 0 && messageIndex < static_cast<int>(messages.size())) {
        selectedMessageIndex = messageIndex;
      }
    }
  }
  
  // Handle scrolling
  if (IsKeyDown(KEY_UP) || GetMouseWheelMove() > 0) {
    scrollOffset += 20;
    if (scrollOffset > 0) scrollOffset = 0;
  }
  if (IsKeyDown(KEY_DOWN) || GetMouseWheelMove() < 0) {
    scrollOffset -= 20;
    float maxScroll = -static_cast<int>(messages.size()) * 20 + bounds.height - 40;
    if (scrollOffset < maxScroll) scrollOffset = maxScroll;
  }
}

void MessagesPanel::render() {
  if (!isVisible) return;
  
  // Draw panel background
  DrawRectangleRec(bounds, Color{240, 240, 240, 255}); // Light gray
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);
  
  // Draw title
  DrawText("MESSAGES", bounds.x + 10, bounds.y + 5, 14, DARKGRAY);
  
  // Draw messages
  float yPos = bounds.y + 25 + scrollOffset;
  for (int i = 0; i < static_cast<int>(messages.size()); ++i) {
    Color textColor = (i == selectedMessageIndex) ? BLUE : DARKGRAY;
    Color bgColor = (i == selectedMessageIndex) ? Color{200, 200, 255, 255} : RAYWHITE;
    
    if (i == selectedMessageIndex) {
      DrawRectangle(bounds.x + 1, static_cast<int>(yPos - 2), static_cast<int>(bounds.width - 2), 18, bgColor);
    }
    
    DrawText(messages[i].c_str(), bounds.x + 5, static_cast<int>(yPos), 12, textColor);
    yPos += 20;
  }
}

// Video Panel Implementation
VideoPanel::VideoPanel(float x, float y, float width, float height)
  : CabanaUIElement(x, y, width, height) {
}

void VideoPanel::update() {
  // Handle play/pause
  if (IsKeyPressed(KEY_SPACE)) {
    isPlaying = !isPlaying;
  }
}

void VideoPanel::render() {
  if (!isVisible) return;
  
  // Draw panel background
  DrawRectangleRec(bounds, Color{50, 50, 50, 255}); // Dark gray
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);
  
  // Draw placeholder video
  Rectangle videoRect = {bounds.x + 5, bounds.y + 5, bounds.width - 10, bounds.height - 40};
  DrawRectangleRec(videoRect, Color{30, 30, 30, 255}); // Video area background
  
  // Draw video placeholder text
  const char* videoText = "VIDEO DISPLAY";
  int textWidth = MeasureText(videoText, 20);
  int textX = bounds.x + (bounds.width - textWidth) / 2;
  int textY = bounds.y + bounds.height / 2 - 10;
  DrawText(videoText, textX, textY, 20, GRAY);
  
  // Draw time display
  char timeText[64];
  snprintf(timeText, sizeof(timeText), "%.2f / %.2f", currentTime, totalTime);
  DrawText(timeText, bounds.x + 10, bounds.y + bounds.height - 30, 14, WHITE);
  
  // Draw play/pause indicator
  const char* statusText = isPlaying ? "PAUSE (SPACE)" : "PLAY (SPACE)";
  int statusWidth = MeasureText(statusText, 14);
  int statusX = bounds.x + bounds.width - statusWidth - 10;
  DrawText(statusText, statusX, bounds.y + bounds.height - 30, 14, WHITE);
}

// Charts Panel Implementation
ChartsPanel::ChartsPanel(float x, float y, float width, float height)
  : CabanaUIElement(x, y, width, height) {
}

void ChartsPanel::update() {
  // Update chart data
}

void ChartsPanel::render() {
  if (!isVisible) return;
  
  // Draw panel background
  DrawRectangleRec(bounds, Color{250, 250, 250, 255}); // Light background
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);
  
  // Draw placeholder chart
  DrawText("CHARTS DISPLAY", bounds.x + 10, bounds.y + 10, 16, DARKGRAY);
  
  // Draw simple chart grid
  Rectangle chartArea = {bounds.x + 50, bounds.y + 30, bounds.width - 60, bounds.height - 40};
  DrawRectangleLines(chartArea.x, chartArea.y, chartArea.width, chartArea.height, GRAY);
  
  // Draw simple data points (placeholder)
  for (int i = 0; i < 10; ++i) {
    float x = chartArea.x + i * (chartArea.width / 10.0f);
    float y = chartArea.y + chartArea.height - (i * 10); // Simple pattern
    DrawCircle(x, y, 3, BLUE);
  }
}

// Menu Bar Implementation
MenuBar::MenuBar(float x, float y, float width, float height)
  : CabanaUIElement(x, y, width, height) {
}

void MenuBar::update() {
  // Update menu state
}

void MenuBar::render() {
  if (!isVisible) return;
  
  // Draw menu bar background
  DrawRectangleRec(bounds, Color{220, 220, 220, 255}); // Light gray menu bar
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, DARKGRAY);
  
  // Draw menu items
  const char* menuItems[] = {"File", "Edit", "View", "Tools", "Help"};
  int itemCount = sizeof(menuItems) / sizeof(menuItems[0]);
  float itemWidth = bounds.width / itemCount;
  
  for (int i = 0; i < itemCount; ++i) {
    Rectangle itemRect = {bounds.x + i * itemWidth, bounds.y, itemWidth, bounds.height};
    DrawText(menuItems[i], itemRect.x + 10, itemRect.y + 5, 14, DARKGRAY);
    
    // Draw separator line
    if (i < itemCount - 1) {
      DrawLineV({itemRect.x + itemWidth - 1, bounds.y}, {itemRect.x + itemWidth - 1, bounds.y + bounds.height}, LIGHTGRAY);
    }
  }
}