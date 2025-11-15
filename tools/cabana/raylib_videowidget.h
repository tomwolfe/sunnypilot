#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tools/cabana/utils/util.h"
#include "tools/replay/logreader.h"
#include "tools/cabana/streams/replaystream.h"
#include "raylib.h"

// Enum for different camera view types
enum class VisionStreamType {
  ROAD = 0,
  DRIVER = 1,
  WIDE_ROAD = 2
};

// Slider implementation for Raylib-based UI
class TimeSlider {
public:
  TimeSlider(float x, float y, float width, float height);
  void update();
  void render();
  void handleInput();
  
  double getCurrentSecond() const { return current_value_ / factor_; }
  void setCurrentSecond(double sec) { current_value_ = sec * factor_; }
  void setTimeRange(double min, double max) { min_ = min * factor_; max_ = max * factor_; }
  
  double getFactor() const { return factor_; }
  bool isHovered() const { return is_hovered_; }
  bool isPressed() const { return is_pressed_; }

private:
  Rectangle bounds_;
  double current_value_ = 0.0;
  double min_ = 0.0;
  double max_ = 100000.0;  // Default to 100 seconds * 1000 factor
  const double factor_ = 1000.0;
  bool is_hovered_ = false;
  bool is_pressed_ = false;
};

// Stream camera view for Raylib-based UI
class StreamCameraView {
public:
  StreamCameraView(std::string stream_name, VisionStreamType stream_type, void* parent = nullptr);
  void render(const Rectangle& bounds);
  void update();
  void handleInput();
  void showPausedOverlay();
  void parseQLog(std::shared_ptr<LogReader> qlog);

private:
  void drawAlert(const Rectangle& rect, const Timeline::Entry& alert);
  void drawThumbnail(const Rectangle& bounds);
  void drawScrubThumbnail(const Rectangle& bounds);
  void drawTime(const Rectangle& rect, double seconds);

  std::string stream_name_;
  VisionStreamType stream_type_;
  bool is_paused_overlay_visible_ = false;
  std::map<uint64_t, Image> thumbnails_;  // Simplified thumbnail storage
  double thumbnail_display_time_ = -1;
};

// Raylib-based VideoWidget
class VideoWidget {
public:
  VideoWidget(void* parent = nullptr);
  void update();
  void render(const Rectangle& bounds);
  void handleInput();
  void showThumbnail(double seconds);

private:
  void createPlaybackController();
  void createSpeedDropdown();
  void loopPlaybackClicked();
  void vipcAvailableStreamsUpdated(const std::set<VisionStreamType>& streams);
  void showRouteInfo();
  std::string formatTime(double sec, bool include_milliseconds = false);

  std::unique_ptr<StreamCameraView> cam_widget_;
  std::unique_ptr<TimeSlider> slider_;
  bool is_playing_ = false;
  bool is_looping_ = false;
  float playback_speed_ = 1.0f;
  double current_time_ = 0.0;
  double total_time_ = 0.0;
  
  Rectangle bounds_;
  bool is_visible_ = true;
  
  // UI state
  std::vector<std::string> speed_options_ = {"0.25x", "0.5x", "1x", "2x", "5x", "10x"};
  int current_speed_index_ = 2;  // Default to 1x
};