#pragma once

#include <memory>
#include <string>

// Include raylib first
#include "raylib.h"

// Now include the capnp-dependent headers but undefine conflicting macros first
// Save color values then undefine
#define SAVED_RED_ RED
#define SAVED_GREEN_ GREEN
#define SAVED_YELLOW_ YELLOW
#define SAVED_WHITE_ WHITE
#define SAVED_GRAY_ GRAY

#undef RED
#undef GREEN
#undef YELLOW
#undef WHITE
#undef GRAY

#include "tools/cabana/utils/util.h"
#include "tools/replay/logreader.h"

// These macros remain undefined for cereal headers, preventing conflicts

// Raylib-based CameraView
class CameraView {
public:
    CameraView(void* parent = nullptr);
    ~CameraView();
    
    void update();
    void render(const Rectangle& bounds);
    void handleInput();
    
    void setVideoPath(const std::string& path);
    void setCurrentTime(double time);
    double getCurrentTime() const { return current_time_; }
    void setPaused(bool paused) { is_paused_ = paused; }
    bool isPaused() const { return is_paused_; }
    
    void togglePlayPause();
    void setPlaybackSpeed(float speed) { playback_speed_ = speed; }
    float getPlaybackSpeed() const { return playback_speed_; }

private:
    void drawVideoFrame(const Rectangle& bounds);
    void drawOverlay(const Rectangle& bounds);
    void drawPlayhead(const Rectangle& bounds);
    
    std::string video_path_;
    double current_time_ = 0.0;
    double total_duration_ = 0.0;
    bool is_paused_ = true;
    float playback_speed_ = 1.0f;
    
    // UI state
    Rectangle bounds_;
    bool is_visible_ = true;
    
    // Frame buffer for video (simplified for this example)
    // In a real implementation, this would be actual video frame data
    Texture2D current_frame_texture_ = {0};
    
    // Overlay controls visibility
    bool show_controls_ = true;
};