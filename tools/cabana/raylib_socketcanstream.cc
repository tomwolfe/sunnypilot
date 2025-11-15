#include "raylib_socketcanstream.h"
#include <thread>
#include <chrono>
#include <algorithm>
#include <stdexcept>  // for std::runtime_error

// Define OPENPILOT_RAYLIB before including raylib to prevent enum conflicts
#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"   // for raylib functions and colors

// SocketCanStream implementation
SocketCanStream::SocketCanStream(const SocketCanStreamConfig& config) : config(config) {
    if (!available()) {
        throw std::runtime_error("SocketCAN is not available on this system");
    }
    
    if (!connect()) {
        throw std::runtime_error("Failed to connect to SocketCAN device");
    }
}

SocketCanStream::~SocketCanStream() {
    // Stop the stream if it's running
    pause(true);
}

bool SocketCanStream::available() {
    // This is a simplified check - in a real implementation, you'd check for SocketCAN availability
    // by attempting to create a socket or checking system capabilities
    return true; // For now, assume available
}

bool SocketCanStream::connect() {
    // This is a simplified implementation
    // In a real implementation, this would create the actual CAN socket connection
    device_connected = true;
    return device_connected;
}

void SocketCanStream::start() {
    if (!device_connected) {
        return;
    }
    
    running = true;
    // In a real implementation, this would start a thread running streamThread
    // std::thread t(&SocketCanStream::streamThread, this);
    // thread_ = std::move(t);
}

void SocketCanStream::pause(bool pause) {
    paused = pause;
    // In a real implementation, this would control the streaming thread
}

std::string SocketCanStream::routeName() const {
    return "Live Streaming From Socket CAN " + config.device;
}

void SocketCanStream::streamThread() {
    // This is a simplified implementation
    // In a real implementation, this would read from the CAN socket
    while (running && !paused) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        
        // Read CAN frames from socket
        // Process the frames
        // Call onUpdateLastMsgs callback
        if (onUpdateLastMsgs) {
            onUpdateLastMsgs();
        }
    }
}

// OpenSocketCanWidget implementation
OpenSocketCanWidget::OpenSocketCanWidget() {
    // Initialize default UI positions
    windowX = 100;
    windowY = 100;
    windowWidth = 400;
    windowHeight = 200;
    
    deviceListX = windowX + 80;
    deviceListY = windowY + 40;
    deviceListWidth = 200;
    deviceListHeight = 25;
    
    refreshButtonX = windowX + 300;
    refreshButtonY = windowY + 40;
    refreshButtonWidth = 80;
    refreshButtonHeight = 25;
    
    // Initialize with some sample devices
    refreshDevices();
}

void OpenSocketCanWidget::render(float x, float y, float width, float height) {
    // Update positions based on where the widget is to be rendered
    windowX = x;
    windowY = y;
    windowWidth = width;
    windowHeight = height;
    
    deviceListX = windowX + 80;
    deviceListY = windowY + 40;
    deviceListWidth = 200;
    deviceListHeight = 25;
    
    refreshButtonX = windowX + 300;
    refreshButtonY = windowY + 40;
    refreshButtonWidth = 80;
    refreshButtonHeight = 25;
    
    // Draw widget background
    DrawRectangle(windowX, windowY, windowWidth, windowHeight, Color{240, 240, 240, 255});
    DrawRectangleLines(windowX, windowY, windowWidth, windowHeight, RAYLIB_DARKGRAY);

    // Draw title
    DrawText("SocketCAN Configuration", windowX + 10, windowY + 10, 14, RAYLIB_DARKGRAY);

    // Draw device label
    DrawText("Device:", windowX + 20, windowY + 45, 12, RAYLIB_DARKGRAY);

    // Draw device selection
    DrawRectangle(deviceListX, deviceListY, deviceListWidth, deviceListHeight, RAYLIB_RAYWHITE);
    DrawRectangleLines(deviceListX, deviceListY, deviceListWidth, deviceListHeight, RAYLIB_DARKGRAY);

    if (!ui_config.availableDevices.empty() && selectedDeviceIndex < ui_config.availableDevices.size()) {
        DrawText(ui_config.availableDevices[selectedDeviceIndex].c_str(), deviceListX + 5, deviceListY + 5, 10, RAYLIB_DARKGRAY);
    } else {
        DrawText("No devices available", deviceListX + 5, deviceListY + 5, 10, RAYLIB_GRAY);
    }

    // Draw refresh button
    DrawRectangle(refreshButtonX, refreshButtonY, refreshButtonWidth, refreshButtonHeight, RAYLIB_GRAY);
    DrawRectangleLines(refreshButtonX, refreshButtonY, refreshButtonWidth, refreshButtonHeight, RAYLIB_DARKGRAY);
    DrawText("Refresh", refreshButtonX + 15, refreshButtonY + 5, 12, RAYLIB_RAYWHITE);

    // Draw dropdown indicator for device selection
    DrawText("v", deviceListX + deviceListWidth - 15, deviceListY + 5, 10, RAYLIB_DARKGRAY);
}

void OpenSocketCanWidget::update() {
    // UI update logic - typically called in main loop
}

void OpenSocketCanWidget::handleInput(float mouse_x, float mouse_y, bool mouse_pressed) {
    if (!mouse_pressed) return;
    
    // Check if refresh button was clicked
    if (mouse_x >= refreshButtonX && mouse_x <= refreshButtonX + refreshButtonWidth &&
        mouse_y >= refreshButtonY && mouse_y <= refreshButtonY + refreshButtonHeight) {
        refreshDevices();
        refreshButtonPressed = true;
    }
    
    // Check if device selection area was clicked (to cycle through devices)
    if (mouse_x >= deviceListX && mouse_x <= deviceListX + deviceListWidth &&
        mouse_y >= deviceListY && mouse_y <= deviceListY + deviceListHeight) {
        if (!ui_config.availableDevices.empty()) {
            selectedDeviceIndex = (selectedDeviceIndex + 1) % ui_config.availableDevices.size();
            if (selectedDeviceIndex < ui_config.availableDevices.size()) {
                ui_config.selectedDevice = ui_config.availableDevices[selectedDeviceIndex];
            }
        }
    }
}

SocketCanStreamConfig OpenSocketCanWidget::getStreamConfig() const {
    SocketCanStreamConfig config;
    config.device = ui_config.selectedDevice;
    config.canFdEnabled = true;
    return config;
}

void OpenSocketCanWidget::refreshDevices() {
    // This would normally query the system for available CAN devices
    // For now, we'll simulate with some example devices
    ui_config.availableDevices = {"can0", "can1", "vcan0", "vcan1"};
    
    if (!ui_config.availableDevices.empty()) {
        selectedDeviceIndex = 0;
        ui_config.selectedDevice = ui_config.availableDevices[0];
    }
}