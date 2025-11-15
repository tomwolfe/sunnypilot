#pragma once

#include <string>

// Simplified DetailWidget implementation using Raylib
class DetailWidget {
public:
  DetailWidget(void* parent = nullptr);
  void showDetail(const std::string& title, const std::string& content);
  void updateContent(const std::string& new_content);
  
private:
  std::string detail_title;
  std::string detail_content;
};