#pragma once

#include <string>

// Forward declaration to avoid Qt dependencies
class QWidgetOrEquivalent;

// Simplified BinaryView implementation without Qt dependencies
class BinaryView {
public:
  BinaryView(QWidgetOrEquivalent* parent = nullptr);
  void showEvent(const std::string& route, int start_time, int end_time);
  void selectItem(int item_id);
  
private:
  // Implementation details would go here
};