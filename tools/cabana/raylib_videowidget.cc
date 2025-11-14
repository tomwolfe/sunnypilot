#include "raylib_videowidget.h"
#include <sstream>
#include <iomanip>

// TimeSlider implementation
TimeSlider::TimeSlider(float x, float y, float width, float height) {
  bounds_ = {x, y, width, height};
}

void TimeSlider::update() {
  Vector2 mouse_pos = GetMousePosition();
  is_hovered_ = CheckCollisionPointRec(mouse_pos, bounds_);
  
  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    if (is_hovered_ || is_pressed_) {
      is_pressed_ = true;
      float relative_x = (mouse_pos.x - bounds_.x) / bounds_.width;
      current_value_ = min_ + relative_x * (max_ - min_);
      if (current_value_ < min_) current_value_ = min_;
      if (current_value_ > max_) current_value_ = max_;
    }
  } else {
    is_pressed_ = false;
  }
}

void TimeSlider::render() {
  // Draw slider background
  DrawRectangleRec(bounds_, LIGHTGRAY);
  
  // Draw filled portion
  float fill_width = ((current_value_ - min_) / (max_ - min_)) * bounds_.width;
  Rectangle fill_rect = {bounds_.x, bounds_.y, fill_width, bounds_.height};
  DrawRectangleRec(fill_rect, BLUE);
  
  // Draw thumb
  float thumb_x = bounds_.x + fill_width - 3;  // Small adjustment for thumb position
  Rectangle thumb = {thumb_x, bounds_.y - 2, 6, bounds_.height + 4};
  DrawRectangleRec(thumb, DARKBLUE);
  
  // Draw time markers (simple implementation)
  DrawLine(bounds_.x, bounds_.y - 5, bounds_.x, bounds_.y + bounds_.height + 5, GRAY);
  DrawLine(bounds_.x + bounds_.width, bounds_.y - 5, bounds_.x + bounds_.width, bounds_.y + bounds_.height + 5, GRAY);
}

void TimeSlider::handleInput() {
  // Input is handled in update method
}

// StreamCameraView implementation
StreamCameraView::StreamCameraView(std::string stream_name, VisionStreamType stream_type, void* parent)
  : stream_name_(stream_name), stream_type_(stream_type) {
  // Initialize camera view
}

void StreamCameraView::render(const Rectangle& bounds) {
  // Draw camera view background
  DrawRectangleRec(bounds, Color{30, 30, 30, 255}); // Dark background
  
  // Draw placeholder video content
  const char* camera_text = "CAMERA VIEW";
  int text_width = MeasureText(camera_text, 20);
  int text_x = bounds.x + (bounds.width - text_width) / 2;
  int text_y = bounds.y + bounds.height / 2 - 10;
  DrawText(camera_text, text_x, text_y, 20, GRAY);
  
  // Draw stream name
  DrawText(stream_name_.c_str(), bounds.x + 10, bounds.y + 10, 14, LIGHTGRAY);
  
  // Draw paused overlay if needed
  if (is_paused_overlay_visible_) {
    DrawRectangleRec(bounds, Color{0, 0, 0, 128}); // Semi-transparent overlay
    const char* paused_text = "PAUSED";
    int p_text_width = MeasureText(paused_text, 30);
    int p_text_x = bounds.x + (bounds.width - p_text_width) / 2;
    int p_text_y = bounds.y + bounds.height / 2 - 15;
    DrawText(paused_text, p_text_x, p_text_y, 30, WHITE);
  }
}

void StreamCameraView::update() {
  // Update camera view state
}

void StreamCameraView::handleInput() {
  // Handle camera view input
}

void StreamCameraView::showPausedOverlay() {
  is_paused_overlay_visible_ = true;
  // Hide after a delay (in a real implementation)
}

void StreamCameraView::parseQLog(std::shared_ptr<LogReader> qlog) {
  // Parse QLog for video frames and thumbnails
  // Implementation would handle video data processing
}

void StreamCameraView::drawAlert(const Rectangle& rect, const Timeline::Entry& alert) {
  // Draw alert overlay on video
  // Implementation would draw alert information on top of video
}

void StreamCameraView::drawThumbnail(const Rectangle& bounds) {
  // Draw preview thumbnail
  // Implementation would handle thumbnail display
}

void StreamCameraView::drawScrubThumbnail(const Rectangle& bounds) {
  // Draw thumbnail during scrubbing
  // Implementation would handle scrubbing preview
}

void StreamCameraView::drawTime(const Rectangle& rect, double seconds) {
  // Draw time overlay on video
  char time_str[32];
  snprintf(time_str, sizeof(time_str), "%02d:%02d:%05.2f", 
           (int)(seconds / 3600), (int)(seconds / 60) % 60, fmod(seconds, 60.0));
  
  int text_width = MeasureText(time_str, 16);
  int text_x = rect.x + (rect.width - text_width) / 2;
  int text_y = rect.y + rect.height - 25;
  
  // Draw background for text to make it more visible
  DrawRectangle(text_x - 5, text_y - 2, text_width + 10, 20, Color{0, 0, 0, 128});
  DrawText(time_str, text_x, text_y, 16, WHITE);
}

// VideoWidget implementation
VideoWidget::VideoWidget(void* parent) {
  // Initialize with default values
  cam_widget_ = std::make_unique<StreamCameraView>("Road Camera", VisionStreamType::ROAD, parent);
}

void VideoWidget::update() {
  if (cam_widget_) {
    cam_widget_->update();
  }
  
  if (slider_) {
    slider_->update();
  }
  
  // Update playback state
  if (is_playing_) {
    current_time_ += GetFrameTime() * playback_speed_;
    if (current_time_ > total_time_) {
      if (is_looping_) {
        current_time_ = 0.0;
      } else {
        is_playing_ = false;
        current_time_ = total_time_;
      }
    }
    
    if (slider_) {
      slider_->setCurrentSecond(current_time_);
    }
  }
}

void VideoWidget::render(const Rectangle& bounds) {
  if (!is_visible_) return;
  
  bounds_ = bounds;
  
  // Draw panel background
  DrawRectangleRec(bounds, Color{50, 50, 50, 255}); // Dark gray
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, LIGHTGRAY);

  // Calculate video area (leaving space for controls)
  Rectangle video_rect = {bounds.x + 5, bounds.y + 5, bounds.width - 10, bounds.height - 60};
  
  // Render camera view
  if (cam_widget_) {
    cam_widget_->render(video_rect);
  }
  
  // Draw time display
  char time_text[64];
  snprintf(time_text, sizeof(time_text), "%s / %s", 
           formatTime(current_time_).c_str(), 
           formatTime(total_time_).c_str());
  DrawText(time_text, bounds.x + 10, bounds.y + bounds.height - 50, 14, WHITE);

  // Draw playback controls
  float control_x = bounds.x + bounds.width - 150;
  float control_y = bounds.y + bounds.height - 50;
  
  // Play/Pause button
  Rectangle play_button = {control_x, control_y, 30, 20};
  DrawRectangleRec(play_button, is_playing_ ? RED : GREEN);
  DrawText(is_playing_ ? "||" : ">", play_button.x + 10, play_button.y + 2, 14, WHITE);
  
  // Loop button
  Rectangle loop_button = {control_x + 35, control_y, 30, 20};
  DrawRectangleRec(loop_button, is_looping_ ? YELLOW : GRAY);
  DrawText("O", loop_button.x + 12, loop_button.y + 2, 14, is_looping_ ? BLACK : WHITE);
  
  // Speed button
  Rectangle speed_button = {control_x + 70, control_y, 40, 20};
  DrawRectangleRec(speed_button, GRAY);
  DrawText(speed_options_[current_speed_index_].c_str(), speed_button.x + 2, speed_button.y + 2, 10, WHITE);
  
  // Fullscreen button
  Rectangle fs_button = {control_x + 115, control_y, 30, 20};
  DrawRectangleRec(fs_button, GRAY);
  DrawText("[]", fs_button.x + 8, fs_button.y + 2, 14, WHITE);
  
  // Draw time slider if it exists
  if (slider_) {
    Rectangle slider_rect = {bounds.x + 50, bounds.y + bounds.height - 25, bounds.width - 100, 15};
    slider_->render();
  }
}

void VideoWidget::handleInput() {
  Vector2 mouse_pos = GetMousePosition();
  
  // Handle play/pause button
  float control_x = bounds_.x + bounds_.width - 150;
  float control_y = bounds_.y + bounds_.height - 50;
  Rectangle play_button = {control_x, control_y, 30, 20};
  
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mouse_pos, play_button)) {
    is_playing_ = !is_playing_;
  }
  
  // Handle loop button
  Rectangle loop_button = {control_x + 35, control_y, 30, 20};
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mouse_pos, loop_button)) {
    is_looping_ = !is_looping_;
  }
  
  // Handle speed button
  Rectangle speed_button = {control_x + 70, control_y, 40, 20};
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mouse_pos, speed_button)) {
    current_speed_index_ = (current_speed_index_ + 1) % speed_options_.size();
    if (current_speed_index_ == 0) current_speed_index_ = 1; // Skip 0.25x, start from 0.5x
    // Update actual playback speed based on string
    std::string speed_str = speed_options_[current_speed_index_];
    if (speed_str == "0.5x") playback_speed_ = 0.5f;
    else if (speed_str == "2x") playback_speed_ = 2.0f;
    else if (speed_str == "5x") playback_speed_ = 5.0f;
    else if (speed_str == "10x") playback_speed_ = 10.0f;
    else playback_speed_ = 1.0f; // Default to 1x
  }
  
  // Handle fullscreen button
  Rectangle fs_button = {control_x + 115, control_y, 30, 20};
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mouse_pos, fs_button)) {
    ToggleFullscreen();
  }
  
  // Handle spacebar for play/pause
  if (IsKeyPressed(KEY_SPACE)) {
    is_playing_ = !is_playing_;
  }
  
  // Handle slider input
  if (slider_) {
    slider_->handleInput();
    if (slider_->isPressed()) {
      current_time_ = slider_->getCurrentSecond();
      is_playing_ = false; // Pause when scrubbing
    }
  }
  
  // Handle camera view input
  if (cam_widget_) {
    cam_widget_->handleInput();
  }
}

void VideoWidget::showThumbnail(double seconds) {
  thumbnail_display_time_ = seconds;
  // Implementation would show a thumbnail at the specified time
}

std::string VideoWidget::formatTime(double sec, bool include_milliseconds) {
  int hours = (int)(sec / 3600);
  int minutes = ((int)(sec / 60)) % 60;
  int seconds_int = ((int)sec) % 60;
  double milliseconds = (sec - (int)sec) * 1000;
  
  std::stringstream ss;
  if (hours > 0) {
    ss << std::setfill('0') << std::setw(2) << hours << ":";
  }
  
  ss << std::setfill('0') << std::setw(2) << minutes << ":"
     << std::setfill('0') << std::setw(2) << seconds_int;
  
  if (include_milliseconds) {
    ss << "." << std::setfill('0') << std::setw(3) << (int)milliseconds;
  }
  
  return ss.str();
}

void VideoWidget::createPlaybackController() {
  // Initialize playback controls
  // In raylib implementation, controls are drawn directly in render method
}

void VideoWidget::createSpeedDropdown() {
  // Speed options are stored in vector, selection handled in input methods
}

void VideoWidget::loopPlaybackClicked() {
  is_looping_ = !is_looping_;
}

void VideoWidget::vipcAvailableStreamsUpdated(const std::set<VisionStreamType>& streams) {
  // Handle available streams update
  // Implementation would update available camera options
}

void VideoWidget::showRouteInfo() {
  // Show route information
  // Implementation would display route details
}