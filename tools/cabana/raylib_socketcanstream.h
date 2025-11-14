#pragma once

#include <string>
#include <vector>
#include <functional>
#include <memory>

// Simplified Stream interface without Qt dependencies
class StreamInterface {
public:
    virtual ~StreamInterface() = default;
    
    // Core stream functions
    virtual void start() = 0;
    virtual void pause(bool pause) = 0;
    virtual bool isPaused() const = 0;
    virtual std::string routeName() const = 0;
    virtual bool liveStreaming() const = 0;
    virtual void seekTo(double ts) = 0;
    
    // Callbacks for data updates
    std::function<void()> onUpdateLastMsgs;
    std::function<void(double)> onSeeking;
    std::function<void(double)> onSeekedTo;
};

// Configuration for SocketCAN stream
struct SocketCanStreamConfig {
    std::string device = "";
    bool canFdEnabled = true;
};

// Simplified SocketCAN Stream without Qt dependencies
class SocketCanStream : public StreamInterface {
public:
    SocketCanStream(const SocketCanStreamConfig& config = {});
    ~SocketCanStream();
    
    void start() override;
    void pause(bool pause) override;
    bool isPaused() const override { return paused; }
    std::string routeName() const override;
    bool liveStreaming() const override { return true; }
    void seekTo(double ts) override {}
    
    // Check if SocketCAN is available on the system
    static bool available();

private:
    void streamThread();
    bool connect();
    
    SocketCanStreamConfig config;
    bool device_connected = false;
    bool paused = false;
    bool running = false;
    // For threading, we would typically use std::thread
    // but for this simplified implementation, we'll just indicate the approach
};

// Configuration structure for the UI widget
struct SocketCanConfigUI {
    std::string selectedDevice;
    std::vector<std::string> availableDevices;
};

// Simplified UI widget for SocketCAN (non-Qt implementation)
class OpenSocketCanWidget {
public:
    OpenSocketCanWidget();
    
    // UI rendering and interaction methods
    void render(float x, float y, float width, float height);
    void update();
    void handleInput(float mouse_x, float mouse_y, bool mouse_pressed);
    
    // Get the stream configuration after UI interaction
    SocketCanStreamConfig getStreamConfig() const;
    
    // Refresh device list
    void refreshDevices();

private:
    // UI state
    SocketCanConfigUI ui_config;
    bool refreshButtonPressed = false;
    int selectedDeviceIndex = 0;
    
    // UI element rectangles for interaction
    float windowX, windowY, windowWidth, windowHeight;
    float deviceListX, deviceListY, deviceListWidth, deviceListHeight;
    float refreshButtonX, refreshButtonY, refreshButtonWidth, refreshButtonHeight;
};