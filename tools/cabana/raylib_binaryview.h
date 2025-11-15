#pragma once

#include <string>

// Forward declaration for Raylib UI element
class RaylibWidget;

// Simplified BinaryView implementation using Raylib
class BinaryView {
public:
  BinaryView(RaylibWidget* parent = nullptr);
  void showEvent(const std::string& route, int start_time, int end_time);
  void selectItem(int item_id);
  
private:
  // Implementation details would go here
};