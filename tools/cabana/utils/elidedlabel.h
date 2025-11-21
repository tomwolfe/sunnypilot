#pragma once

#include <string>
#include <functional>

#include "third_party/raylib/include/raylib.h"

// Raylib-based ElidedLabel for text that shows "..." when too long to fit
class ElidedLabel {
public:
  explicit ElidedLabel(void* parent = nullptr);
  explicit ElidedLabel(const std::string &text, void* parent = nullptr);

  // Set the text to be displayed
  void setText(const std::string &text);
  std::string getText() const { return text_; }

  // Draw the elided text within the given bounds
  void render(const Rectangle &bounds, int fontSize = 10, Color color = {0, 0, 0, 255});

  // Check if the label was clicked
  bool wasClicked();

  // Event callback
  std::function<void()> onClick;

private:
  std::string text_;
  std::string elided_text_;
  Rectangle bounds_;
  bool last_click_state_ = false;
};
