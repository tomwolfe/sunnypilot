#include "raylib_replaystream.h"
#include <thread>
#include <chrono>
#include <algorithm>

// ReplayStream implementation
ReplayStream::ReplayStream() {
    // Initialize default values
    route_name = "No route loaded";
    car_fingerprint = "Unknown";
    min_seconds = 0.0;
    max_seconds = 0.0;
    paused = true;
    running = false;
}

ReplayStream::~ReplayStream() {
    // Stop the stream if it's running
    pause(true);
}

void ReplayStream::start() {
    if (route_name == "No route loaded") {
        return; // Can't start without a route
    }
    
    running = true;
    paused = false;
    // In a real implementation, this would start a thread running streamThread
    // std::thread t(&ReplayStream::streamThread, this);
    // thread_ = std::move(t);
}

void ReplayStream::pause(bool pause) {
    paused = pause;
    // In a real implementation, this would control the streaming thread
}

void ReplayStream::seekTo(double ts) {
    if (onSeeking) onSeeking(ts);
    // In a real implementation, seek to the specified time
    // After seek is complete:
    if (onSeekedTo) onSeekedTo(ts);
}

bool ReplayStream::loadRoute(const std::string &route, const std::string &data_dir, 
                            uint32_t replay_flags, bool auto_source) {
    // This is a simplified implementation
    // In a real implementation, this would initialize the replay system
    route_name = route;
    // Set other properties based on the loaded route
    
    // For now, return true to indicate success
    return true;
}

bool ReplayStream::eventFilter(void* event) {
    // This is a simplified event filter
    // In a real implementation, this would process replay events
    
    // Periodically call onUpdateLastMsgs callback
    if (onUpdateLastMsgs) {
        onUpdateLastMsgs();
    }
    
    return true;
}

void ReplayStream::streamThread() {
    // This is a simplified implementation
    // In a real implementation, this would handle replay data
    while (running && !paused) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Process replay data
        // Call callbacks as needed
    }
}

// OpenReplayWidget implementation
OpenReplayWidget::OpenReplayWidget() {
    // Initialize default UI positions
    windowX = 150;
    windowY = 100;
    windowWidth = 600;
    windowHeight = 300;
    
    routeInputX = windowX + 100;
    routeInputY = windowY + 40;
    routeInputWidth = 300;
    routeInputHeight = 25;
    
    browseLocalButtonX = windowX + 420;
    browseLocalButtonY = windowY + 40;
    browseLocalButtonWidth = 80;
    browseLocalButtonHeight = 25;
    
    browseRemoteButtonX = windowX + 510;
    browseRemoteButtonY = windowY + 40;
    browseRemoteButtonWidth = 80;
    browseRemoteButtonHeight = 25;
    
    roadCameraX = windowX + 20;
    roadCameraY = windowY + 80;
    
    driverCameraX = windowX + 150;
    driverCameraY = windowY + 80;
    
    wideRoadCameraX = windowX + 280;
    wideRoadCameraY = windowY + 80;
    
    openButtonX = windowX + 250;
    openButtonY = windowY + 250;
    openButtonWidth = 100;
    openButtonHeight = 30;
    
    // Initialize with a sample route
    ui_config.route = "Enter route name or browse...";
    ui_config.lastRouteDir = "/home/user/routes";
}

void OpenReplayWidget::render(float x, float y, float width, float height) {
    // Update positions based on where the widget is to be rendered
    windowX = x;
    windowY = y;
    windowWidth = width;
    windowHeight = height;
    
    routeInputX = windowX + 100;
    routeInputY = windowY + 40;
    routeInputWidth = 300;
    routeInputHeight = 25;
    
    browseLocalButtonX = windowX + 420;
    browseLocalButtonY = windowY + 40;
    browseLocalButtonWidth = 80;
    browseLocalButtonHeight = 25;
    
    browseRemoteButtonX = windowX + 510;
    browseRemoteButtonY = windowY + 40;
    browseRemoteButtonWidth = 80;
    browseRemoteButtonHeight = 25;
    
    roadCameraX = windowX + 20;
    roadCameraY = windowY + 80;
    
    driverCameraX = windowX + 150;
    driverCameraY = windowY + 80;
    
    wideRoadCameraX = windowX + 280;
    wideRoadCameraY = windowY + 80;
    
    openButtonX = windowX + 250;
    openButtonY = windowY + 250;
    openButtonWidth = 100;
    openButtonHeight = 30;
    
    // Draw widget background
    DrawRectangle(windowX, windowY, windowWidth, windowHeight, Color{240, 240, 240, 255});
    DrawRectangleLines(windowX, windowY, windowWidth, windowHeight, BLACK);
    
    // Draw title
    DrawText("Replay Stream Configuration", windowX + 10, windowY + 10, 16, BLACK);
    
    // Draw route input
    DrawText("Route:", windowX + 20, windowY + 45, 12, BLACK);
    DrawRectangle(routeInputX, routeInputY, routeInputWidth, routeInputHeight, WHITE);
    DrawRectangleLines(routeInputX, routeInputY, routeInputWidth, routeInputHeight, BLACK);
    DrawText(ui_config.route.c_str(), routeInputX + 5, routeInputY + 5, 10, inputActive ? BLACK : GRAY);
    
    // Draw browse buttons
    DrawRectangle(browseLocalButtonX, browseLocalButtonY, browseLocalButtonWidth, browseLocalButtonHeight, GRAY);
    DrawRectangleLines(browseLocalButtonX, browseLocalButtonY, browseLocalButtonWidth, browseLocalButtonHeight, BLACK);
    DrawText("Local...", browseLocalButtonX + 10, browseLocalButtonY + 5, 10, WHITE);
    
    DrawRectangle(browseRemoteButtonX, browseRemoteButtonY, browseRemoteButtonWidth, browseRemoteButtonHeight, GRAY);
    DrawRectangleLines(browseRemoteButtonX, browseRemoteButtonY, browseRemoteButtonWidth, browseRemoteButtonHeight, BLACK);
    DrawText("Remote...", browseRemoteButtonX + 10, browseRemoteButtonY + 5, 10, WHITE);
    
    // Draw camera options
    // Road Camera
    DrawText("Road Camera", roadCameraX, roadCameraY, 12, BLACK);
    Rectangle roadCameraBox = {roadCameraX - 20, roadCameraY, 15, 15};
    DrawRectangleRec(roadCameraBox, WHITE);
    DrawRectangleLines(roadCameraBox.x, roadCameraBox.y, roadCameraBox.width, roadCameraBox.height, BLACK);
    if (ui_config.roadCamera) {
        DrawRectangle(roadCameraBox.x + 2, roadCameraBox.y + 2, 
                     roadCameraBox.width - 4, roadCameraBox.height - 4, BLACK);
    }
    
    // Driver Camera
    DrawText("Driver Camera", driverCameraX, driverCameraY, 12, BLACK);
    Rectangle driverCameraBox = {driverCameraX - 20, driverCameraY, 15, 15};
    DrawRectangleRec(driverCameraBox, WHITE);
    DrawRectangleLines(driverCameraBox.x, driverCameraBox.y, driverCameraBox.width, driverCameraBox.height, BLACK);
    if (ui_config.driverCamera) {
        DrawRectangle(driverCameraBox.x + 2, driverCameraBox.y + 2, 
                     driverCameraBox.width - 4, driverCameraBox.height - 4, BLACK);
    }
    
    // Wide Road Camera
    DrawText("Wide Road Camera", wideRoadCameraX, wideRoadCameraY, 12, BLACK);
    Rectangle wideRoadCameraBox = {wideRoadCameraX - 20, wideRoadCameraY, 15, 15};
    DrawRectangleRec(wideRoadCameraBox, WHITE);
    DrawRectangleLines(wideRoadCameraBox.x, wideRoadCameraBox.y, wideRoadCameraBox.width, wideRoadCameraBox.height, BLACK);
    if (ui_config.wideRoadCamera) {
        DrawRectangle(wideRoadCameraBox.x + 2, wideRoadCameraBox.y + 2, 
                     wideRoadCameraBox.width - 4, wideRoadCameraBox.height - 4, BLACK);
    }
    
    // Draw open button
    DrawRectangle(openButtonX, openButtonY, openButtonWidth, openButtonHeight, GREEN);
    DrawRectangleLines(openButtonX, openButtonY, openButtonWidth, openButtonHeight, BLACK);
    DrawText("Open", openButtonX + 30, openButtonY + 8, 14, WHITE);
}

void OpenReplayWidget::update() {
    // UI update logic - typically called in main loop
}

void OpenReplayWidget::handleInput(float mouse_x, float mouse_y, bool mouse_pressed, bool key_pressed, int key) {
    if (mouse_pressed) {
        // Check if route input was clicked
        if (mouse_x >= routeInputX && mouse_x <= routeInputX + routeInputWidth &&
            mouse_y >= routeInputY && mouse_y <= routeInputY + routeInputHeight) {
            inputActive = true;
        } else {
            inputActive = false;
        }
        
        // Check browse local button
        if (mouse_x >= browseLocalButtonX && mouse_x <= browseLocalButtonX + browseLocalButtonWidth &&
            mouse_y >= browseLocalButtonY && mouse_y <= browseLocalButtonY + browseLocalButtonHeight) {
            // In a real implementation, this would open a file dialog
            ui_config.route = "Selected local route...";
            browseLocalPressed = true;
        }
        
        // Check browse remote button
        if (mouse_x >= browseRemoteButtonX && mouse_x <= browseRemoteButtonX + browseRemoteButtonWidth &&
            mouse_y >= browseRemoteButtonY && mouse_y <= browseRemoteButtonY + browseRemoteButtonHeight) {
            // In a real implementation, this would open a remote route selection dialog
            ui_config.route = "Selected remote route...";
            browseRemotePressed = true;
        }
        
        // Check camera options
        Rectangle roadCameraBox = {roadCameraX - 20, roadCameraY, 15, 15};
        if (mouse_x >= roadCameraBox.x && mouse_x <= roadCameraBox.x + roadCameraBox.width &&
            mouse_y >= roadCameraBox.y && mouse_y <= roadCameraBox.y + roadCameraBox.height) {
            ui_config.roadCamera = !ui_config.roadCamera;
        }
        
        Rectangle driverCameraBox = {driverCameraX - 20, driverCameraY, 15, 15};
        if (mouse_x >= driverCameraBox.x && mouse_x <= driverCameraBox.x + driverCameraBox.width &&
            mouse_y >= driverCameraBox.y && mouse_y <= driverCameraBox.y + driverCameraBox.height) {
            ui_config.driverCamera = !ui_config.driverCamera;
        }
        
        Rectangle wideRoadCameraBox = {wideRoadCameraX - 20, wideRoadCameraY, 15, 15};
        if (mouse_x >= wideRoadCameraBox.x && mouse_x <= wideRoadCameraBox.x + wideRoadCameraBox.width &&
            mouse_y >= wideRoadCameraBox.y && mouse_y <= wideRoadCameraBox.y + wideRoadCameraBox.height) {
            ui_config.wideRoadCamera = !ui_config.wideRoadCamera;
        }
        
        // Check open button
        if (mouse_x >= openButtonX && mouse_x <= openButtonX + openButtonWidth &&
            mouse_y >= openButtonY && mouse_y <= openButtonY + openButtonHeight) {
            openPressed = true;
        }
    }
    
    // Handle text input if route input is active
    if (inputActive && key_pressed) {
        if (key >= 32 && key <= 126) { // Printable ASCII characters
            inputBuffer += static_cast<char>(key);
            ui_config.route = inputBuffer;
        } else if (key == 259) { // Backspace (KEY_BACKSPACE)
            if (!inputBuffer.empty()) {
                inputBuffer.pop_back();
                ui_config.route = inputBuffer;
            }
        }
    }
}

ReplayStream* OpenReplayWidget::open() {
    // Validate route format - simplified check
    bool isValidFormat = ui_config.route.length() > 0 && ui_config.route != "Enter route name or browse...";
    
    if (!isValidFormat) {
        // In a real implementation, this would show a warning
        return nullptr;
    }
    
    try {
        auto replay_stream = std::make_unique<ReplayStream>();
        
        uint32_t flags = 0;
        if (ui_config.driverCamera) flags |= 2; // Assuming REPLAY_FLAG_DCAM = 2
        if (ui_config.wideRoadCamera) flags |= 4; // Assuming REPLAY_FLAG_ECAM = 4
        if (flags == 0 && !ui_config.roadCamera) flags = 1; // REPLAY_FLAG_NO_VIPC = 1
        
        if (replay_stream->loadRoute(ui_config.route)) {
            return replay_stream.release();
        }
    } catch (...) {
        // Error handling
        return nullptr;
    }
    
    return nullptr;
}