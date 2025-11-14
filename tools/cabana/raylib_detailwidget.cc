#include "raylib_detailwidget.h"
#include <string>

DetailWidget::DetailWidget(void* parent) {
  // Constructor implementation
}

void DetailWidget::showDetail(const std::string& title, const std::string& content) {
  detail_title = title;
  detail_content = content;
  // In a full Raylib implementation, this would render the detail view
}

void DetailWidget::updateContent(const std::string& new_content) {
  detail_content = new_content;
  // Update the displayed content
}