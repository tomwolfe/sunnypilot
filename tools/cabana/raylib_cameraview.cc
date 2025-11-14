#include "raylib_cameraview.h"

CameraView::CameraView(void* parent) {
    // Initialize with default values
}

CameraView::~CameraView() {
    // Clean up texture if it exists
    if (current_frame_texture_.id != 0) {
        UnloadTexture(current_frame_texture_);
    }
}

void CameraView::update() {
    // Update video playback based on time
    if (!is_paused_) {
        current_time_ += GetFrameTime() * playback_speed_;
        if (current_time_ > total_duration_) {
            current_time_ = total_duration_;
            is_paused_ = true;  // Stop at end
        }
    }
}

void CameraView::render(const Rectangle& bounds) {
    if (!is_visible_) return;
    
    bounds_ = bounds;
    
    // Draw camera view background
    DrawRectangleRec(bounds, Color{30, 30, 30, 255}); // Dark background
    
    // Draw video frame
    drawVideoFrame(bounds);
    
    // Draw overlay with playback controls
    if (show_controls_) {
        drawOverlay(bounds);
    }
    
    // Draw playhead indicator
    drawPlayhead(bounds);
}

void CameraView::drawVideoFrame(const Rectangle& bounds) {
    // Draw a placeholder if no actual video texture
    if (current_frame_texture_.id == 0) {
        // Draw a simple pattern as video placeholder
        for (int y = 0; y < 8; y++) {
            for (int x = 0; x < 8; x++) {
                Color color = (x + y) % 2 == 0 ? Color{50, 50, 50, 255} : Color{100, 100, 100, 255};
                Rectangle tile = {
                    bounds.x + x * (bounds.width / 8.0f),
                    bounds.y + y * (bounds.height / 8.0f),
                    bounds.width / 8.0f,
                    bounds.height / 8.0f
                };
                DrawRectangleRec(tile, color);
            }
        }
        
        // Draw video label
        const char* video_label = "VIDEO FEED";
        int text_width = MeasureText(video_label, 20);
        int text_x = bounds.x + (bounds.width - text_width) / 2;
        int text_y = bounds.y + bounds.height / 2 - 10;
        DrawText(video_label, text_x, text_y, 20, GRAY);
    } else {
        // Draw actual texture
        Rectangle dest_rect = {bounds.x, bounds.y, bounds.width, bounds.height};
        Rectangle source_rect = {0, 0, (float)current_frame_texture_.width, (float)current_frame_texture_.height};
        DrawTextureRec(current_frame_texture_, source_rect, (Vector2){dest_rect.x, dest_rect.y}, WHITE);
    }
}

void CameraView::drawOverlay(const Rectangle& bounds) {
    // Draw time display overlay at the top
    char time_str[64];
    snprintf(time_str, sizeof(time_str), "%.2f / %.2f", current_time_, total_duration_);
    int text_width = MeasureText(time_str, 14);
    DrawRectangle(bounds.x + 10, bounds.y + 10, text_width + 10, 24, Color{0, 0, 0, 128});
    DrawText(time_str, bounds.x + 15, bounds.y + 15, 14, WHITE);
    
    // Draw playback controls at the bottom
    Rectangle controls_rect = {bounds.x, bounds.y + bounds.height - 40, bounds.width, 40};
    DrawRectangleRec(controls_rect, Color{0, 0, 0, 180});
    
    // Play/Pause button
    Rectangle play_button = {bounds.x + 20, bounds.y + bounds.height - 30, 30, 20};
    DrawRectangleRec(play_button, is_paused_ ? GREEN : RED);
    DrawText(is_paused_ ? "▶" : "⏸", play_button.x + 8, play_button.y + 2, 14, WHITE);
    
    // Speed indicator
    char speed_str[16];
    snprintf(speed_str, sizeof(speed_str), "%.1fx", playback_speed_);
    DrawText(speed_str, bounds.x + 60, bounds.y + bounds.height - 25, 12, WHITE);
    
    // Fullscreen button
    Rectangle fs_button = {bounds.x + bounds.width - 40, bounds.y + bounds.height - 30, 30, 20};
    DrawRectangleRec(fs_button, Color{70, 70, 70, 255});
    DrawText("⛶", fs_button.x + 8, fs_button.y + 2, 14, WHITE);
}

void CameraView::drawPlayhead(const Rectangle& bounds) {
    // Draw timeline at the bottom
    Rectangle timeline = {bounds.x + 50, bounds.y + bounds.height - 15, bounds.width - 100, 5};
    DrawRectangleRec(timeline, Color{100, 100, 100, 255});
    
    // Draw current time indicator
    if (total_duration_ > 0) {
        float position = (current_time_ / total_duration_) * timeline.width;
        Vector2 line_start = {timeline.x + position, timeline.y};
        Vector2 line_end = {timeline.x + position, timeline.y + timeline.height + 10};
        DrawLineV(line_start, line_end, YELLOW);
    }
}

void CameraView::handleInput() {
    Vector2 mouse_pos = GetMousePosition();
    
    // Handle play/pause button
    Rectangle play_button = {bounds_.x + 20, bounds_.y + bounds_.height - 30, 30, 20};
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mouse_pos, play_button)) {
        togglePlayPause();
    }
    
    // Handle spacebar for play/pause
    if (IsKeyPressed(KEY_SPACE)) {
        togglePlayPause();
    }
    
    // Handle timeline scrubbing
    Rectangle timeline = {bounds_.x + 50, bounds_.y + bounds_.height - 15, bounds_.width - 100, 15};
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && CheckCollisionPointRec(mouse_pos, timeline)) {
        float relative_pos = (mouse_pos.x - timeline.x) / timeline.width;
        current_time_ = relative_pos * total_duration_;
        if (current_time_ < 0) current_time_ = 0;
        if (current_time_ > total_duration_) current_time_ = total_duration_;
    }
    
    // Handle speed change with mouse wheel over video area
    if (CheckCollisionPointRec(mouse_pos, (Rectangle){bounds_.x, bounds_.y, bounds_.width, bounds_.height - 40})) {
        float wheel_move = GetMouseWheelMove();
        if (wheel_move != 0) {
            float new_speed = playback_speed_ + (wheel_move > 0 ? 0.5f : -0.5f);
            // Constrain speed between 0.1x and 10x
            if (new_speed < 0.1f) new_speed = 0.1f;
            if (new_speed > 10.0f) new_speed = 10.0f;
            playback_speed_ = new_speed;
        }
    }
}

void CameraView::setVideoPath(const std::string& path) {
    video_path_ = path;
    // In a real implementation, this would load the video and determine duration
    total_duration_ = 100.0;  // Dummy duration for testing
}

void CameraView::setCurrentTime(double time) {
    current_time_ = time;
    if (current_time_ < 0) current_time_ = 0;
    if (current_time_ > total_duration_) current_time_ = total_duration_;
}

void CameraView::togglePlayPause() {
    is_paused_ = !is_paused_;
}