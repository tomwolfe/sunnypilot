#pragma once

#include <string>
#include <vector>
#include <functional>
#include <memory>

#include "raylib.h"  // For UI elements if needed

// Forward declarations for replay-related classes
// In a real implementation, these would come from the replay system
struct LogReader;

// Simplified Replay Stream interface using Raylib
class ReplayStream {
public:
    ReplayStream();
    ~ReplayStream();
    
    void start();
    void pause(bool pause);
    bool isPaused() const { return paused; }
    std::string routeName() const { return route_name; }
    bool liveStreaming() const { return false; }
    void seekTo(double ts);
    
    // Load route - returns true if successful
    bool loadRoute(const std::string &route, const std::string &data_dir = "", 
                   uint32_t replay_flags = 0, bool auto_source = false);
    
    // Getters
    double minSeconds() const { return min_seconds; }
    double maxSeconds() const { return max_seconds; }
    std::string carFingerprint() const { return car_fingerprint; }
    
    // Callbacks for events
    std::function<void()> onUpdateLastMsgs;
    std::function<void(double)> onSeeking;
    std::function<void(double)> onSeekedTo;
    std::function<void(std::shared_ptr<LogReader>)> onQLogLoaded;

private:
    void streamThread();
    bool eventFilter(void* event); // Simplified event filter
    
    std::string route_name;
    std::string car_fingerprint;
    double min_seconds = 0.0;
    double max_seconds = 0.0;
    bool paused = false;
    bool running = false;
};

// Configuration structure for the replay UI widget
struct ReplayConfigUI {
    std::string route;
    bool roadCamera = true;
    bool driverCamera = false;
    bool wideRoadCamera = false;
    std::string lastRouteDir;
    std::vector<std::string> recentRoutes;
};

// Simplified UI widget for Replay stream selection using Raylib
class OpenReplayWidget {
public:
    OpenReplayWidget();
    
    // UI rendering and interaction methods
    void render(float x, float y, float width, float height);
    void update();
    void handleInput(float mouse_x, float mouse_y, bool mouse_pressed, bool key_pressed, int key);
    
    // Get the stream configuration after UI interaction
    ReplayConfigUI getReplayConfig() const { return ui_config; }
    
    // Open the stream based on UI configuration
    ReplayStream* open();

private:
    // UI state
    ReplayConfigUI ui_config;
    bool browseLocalPressed = false;
    bool browseRemotePressed = false;
    bool openPressed = false;
    
    // UI element rectangles for interaction
    float windowX, windowY, windowWidth, windowHeight;
    float routeInputX, routeInputY, routeInputWidth, routeInputHeight;
    float browseLocalButtonX, browseLocalButtonY, browseLocalButtonWidth, browseLocalButtonHeight;
    float browseRemoteButtonX, browseRemoteButtonY, browseRemoteButtonWidth, browseRemoteButtonHeight;
    float roadCameraX, roadCameraY;
    float driverCameraX, driverCameraY;
    float wideRoadCameraX, wideRoadCameraY;
    float openButtonX, openButtonY, openButtonWidth, openButtonHeight;
    
    // Text input handling
    std::string inputBuffer;
    int cursorPosition = 0;
    bool inputActive = false;
};