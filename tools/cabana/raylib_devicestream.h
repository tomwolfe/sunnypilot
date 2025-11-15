#pragma once

#include <string>
#include <functional>

#include "raylib.h"  // For UI elements if needed

// Simplified Device Stream interface using Raylib
class DeviceStream {
public:
    DeviceStream(const std::string& address = "");
    ~DeviceStream();
    
    void start();
    void pause(bool pause);
    bool isPaused() const { return paused; }
    std::string routeName() const;
    bool liveStreaming() const { return true; }
    void seekTo(double ts) {}
    
    // Callbacks for events
    std::function<void()> onUpdateLastMsgs;

private:
    void streamThread();
    
    std::string zmq_address;
    bool paused = false;
    bool running = false;
};

// Configuration structure for the device UI widget
struct DeviceConfigUI {
    std::string ipAddress = "127.0.0.1";
    bool useZmq = true;  // true for ZMQ, false for MSGQ
    bool validIpAddress = true;
};

// Simplified UI widget for Device stream selection using Raylib
class OpenDeviceWidget {
public:
    OpenDeviceWidget();
    
    // UI rendering and interaction methods
    void render(float x, float y, float width, float height);
    void update();
    void handleInput(float mouse_x, float mouse_y, bool mouse_pressed, bool key_pressed, int key);
    
    // Get the stream configuration after UI interaction
    DeviceConfigUI getDeviceConfig() const { return ui_config; }
    
    // Open the stream based on UI configuration
    DeviceStream* open();

private:
    // UI state
    DeviceConfigUI ui_config;
    bool openPressed = false;
    int selectedProtocol = 1; // 0 for MSGQ, 1 for ZMQ
    
    // UI element rectangles for interaction
    float windowX, windowY, windowWidth, windowHeight;
    float msgqRadioX, msgqRadioY;
    float zmqRadioX, zmqRadioY;
    float ipInputX, ipInputY, ipInputWidth, ipInputHeight;
    float openButtonX, openButtonY, openButtonWidth, openButtonHeight;
    
    // Text input handling
    std::string inputBuffer;
    bool inputActive = false;
    
    // Validation helper
    bool isValidIpAddress(const std::string& ip) const;
};