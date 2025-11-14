#pragma once

#include <memory>
#include <string>

#include "raylib.h"
#include "tools/cabana/utils/util.h"
#include "tools/replay/logreader.h"

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