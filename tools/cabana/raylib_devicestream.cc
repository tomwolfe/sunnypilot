#include "raylib_devicestream.h"
#include <thread>
#include <chrono>
#include <regex>

// DeviceStream implementation
DeviceStream::DeviceStream(const std::string& address) : zmq_address(address) {
    // Initialize default values
    paused = true;
    running = false;
}

DeviceStream::~DeviceStream() {
    // Stop the stream if it's running
    pause(true);
}

void DeviceStream::start() {
    running = true;
    paused = false;
    // In a real implementation, this would start a thread running streamThread
    // std::thread t(&DeviceStream::streamThread, this);
    // thread_ = std::move(t);
}

void DeviceStream::pause(bool pause) {
    paused = pause;
    // In a real implementation, this would control the streaming thread
}

std::string DeviceStream::routeName() const {
    std::string address = zmq_address.empty() ? "127.0.0.1" : zmq_address;
    return "Live Streaming From " + address;
}

void DeviceStream::streamThread() {
    // This is a simplified implementation
    // In a real implementation, this would connect to the device via ZMQ/MSGQ
    while (running && !paused) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Receive messages from device
        // Process the messages
        // Call onUpdateLastMsgs callback
        if (onUpdateLastMsgs) {
            onUpdateLastMsgs();
        }
    }
}

// OpenDeviceWidget implementation
OpenDeviceWidget::OpenDeviceWidget() {
    // Initialize default UI positions
    windowX = 200;
    windowY = 150;
    windowWidth = 450;
    windowHeight = 200;
    
    msgqRadioX = windowX + 30;
    msgqRadioY = windowY + 40;
    
    zmqRadioX = windowX + 30;
    zmqRadioY = windowY + 80;
    
    ipInputX = windowX + 100;
    ipInputY = windowY + 80;
    ipInputWidth = 200;
    ipInputHeight = 25;
    
    openButtonX = windowX + 175;
    openButtonY = windowY + 150;
    openButtonWidth = 100;
    openButtonHeight = 30;
    
    // Initialize with default values
    inputBuffer = ui_config.ipAddress;
}

void OpenDeviceWidget::render(float x, float y, float width, float height) {
    // Update positions based on where the widget is to be rendered
    windowX = x;
    windowY = y;
    windowWidth = width;
    windowHeight = height;
    
    msgqRadioX = windowX + 30;
    msgqRadioY = windowY + 40;
    
    zmqRadioX = windowX + 30;
    zmqRadioY = windowY + 80;
    
    ipInputX = windowX + 100;
    ipInputY = windowY + 80;
    ipInputWidth = 200;
    ipInputHeight = 25;
    
    openButtonX = windowX + 175;
    openButtonY = windowY + 150;
    openButtonWidth = 100;
    openButtonHeight = 30;
    
    // Draw widget background
    DrawRectangle(windowX, windowY, windowWidth, windowHeight, Color{240, 240, 240, 255});
    DrawRectangleLines(windowX, windowY, windowWidth, windowHeight, BLACK);
    
    // Draw title
    DrawText("Device Stream Configuration", windowX + 10, windowY + 10, 16, BLACK);
    
    // Draw MSGQ radio button
    DrawText("MSGQ", msgqRadioX + 20, msgqRadioY + 3, 12, BLACK);
    Rectangle msgqCircle = {msgqRadioX, msgqRadioY, 15, 15};
    DrawCircleLines(msgqCircle.x + 7, msgqRadioY + 7, 7, BLACK);
    DrawCircle(msgqCircle.x + 7, msgqRadioY + 7, 3, selectedProtocol == 0 ? BLACK : RAYWHITE);
    
    // Draw ZMQ radio button and IP input
    DrawText("ZMQ", zmqRadioX + 20, zmqRadioY + 3, 12, BLACK);
    Rectangle zmqCircle = {zmqRadioX, zmqRadioY, 15, 15};
    DrawCircleLines(zmqCircle.x + 7, zmqRadioY + 7, 7, BLACK);
    DrawCircle(zmqCircle.x + 7, zmqRadioY + 7, 3, selectedProtocol == 1 ? BLACK : RAYWHITE);
    
    // Draw IP address input
    DrawText("IP Address:", windowX + 100, windowY + 60, 12, BLACK);
    DrawRectangle(ipInputX, ipInputY, ipInputWidth, ipInputHeight, WHITE);
    DrawRectangleLines(ipInputX, ipInputY, ipInputWidth, ipInputHeight, ui_config.validIpAddress ? BLACK : RED);
    DrawText(inputBuffer.c_str(), ipInputX + 5, ipInputY + 5, 10, inputActive ? BLACK : GRAY);
    DrawText("Enter device IP Address", ipInputX + 5, ipInputY - 15, 10, GRAY);
    
    // Draw open button
    DrawRectangle(openButtonX, openButtonY, openButtonWidth, openButtonHeight, GREEN);
    DrawRectangleLines(openButtonX, openButtonY, openButtonWidth, openButtonHeight, BLACK);
    DrawText("Open", openButtonX + 35, openButtonY + 8, 14, WHITE);
    
    // Show validation error if needed
    if (!ui_config.validIpAddress) {
        DrawText("Invalid IP address format", windowX + 100, windowY + 110, 10, RED);
    }
}

void OpenDeviceWidget::update() {
    // UI update logic - typically called in main loop
}

void OpenDeviceWidget::handleInput(float mouse_x, float mouse_y, bool mouse_pressed, bool key_pressed, int key) {
    if (mouse_pressed) {
        // Check MSGQ radio button
        Rectangle msgqCircle = {msgqRadioX, msgqRadioY, 15, 15};
        if (mouse_x >= msgqCircle.x && mouse_x <= msgqCircle.x + msgqCircle.width &&
            mouse_y >= msgqCircle.y && mouse_y <= msgqCircle.y + msgqCircle.height) {
            selectedProtocol = 0;
            ui_config.useZmq = false;
        }
        
        // Check ZMQ radio button
        Rectangle zmqCircle = {zmqRadioX, zmqRadioY, 15, 15};
        if (mouse_x >= zmqCircle.x && mouse_x <= zmqCircle.x + zmqCircle.width &&
            mouse_y >= zmqCircle.y && mouse_y <= zmqCircle.y + zmqCircle.height) {
            selectedProtocol = 1;
            ui_config.useZmq = true;
        }
        
        // Check IP input field
        if (mouse_x >= ipInputX && mouse_x <= ipInputX + ipInputWidth &&
            mouse_y >= ipInputY && mouse_y <= ipInputY + ipInputHeight) {
            if (selectedProtocol == 1) { // Only allow input when ZMQ is selected
                inputActive = true;
            }
        } else if (!(mouse_x >= msgqCircle.x && mouse_x <= msgqCircle.x + msgqCircle.width &&
                     mouse_y >= msgqCircle.y && mouse_y <= msgqCircle.y + msgqCircle.height) &&
                   !(mouse_x >= zmqCircle.x && mouse_x <= zmqCircle.x + zmqCircle.width &&
                     mouse_y >= zmqCircle.y && mouse_y <= zmqCircle.y + zmqCircle.height)) {
            inputActive = false;
        }
        
        // Check open button
        if (mouse_x >= openButtonX && mouse_x <= openButtonX + openButtonWidth &&
            mouse_y >= openButtonY && mouse_y <= openButtonY + openButtonHeight) {
            openPressed = true;
        }
    }
    
    // Handle text input if IP input is active and ZMQ is selected
    if (inputActive && selectedProtocol == 1 && key_pressed) {
        if (key >= 32 && key <= 126) { // Printable ASCII characters
            inputBuffer += static_cast<char>(key);
        } else if (key == 259) { // Backspace (KEY_BACKSPACE)
            if (!inputBuffer.empty()) {
                inputBuffer.pop_back();
            }
        }
    }
}

DeviceStream* OpenDeviceWidget::open() {
    // Set the IP address based on input
    if (selectedProtocol == 1) { // ZMQ selected
        ui_config.ipAddress = inputBuffer.empty() ? "127.0.0.1" : inputBuffer;
        ui_config.validIpAddress = isValidIpAddress(ui_config.ipAddress);
        
        if (!ui_config.validIpAddress) {
            return nullptr; // Invalid IP address
        }
    } else { // MSGQ selected
        ui_config.ipAddress = ""; // Empty IP means use MSGQ
    }
    
    try {
        std::string ip = ui_config.ipAddress.empty() ? "127.0.0.1" : ui_config.ipAddress;
        return new DeviceStream(ui_config.useZmq ? ip : "");
    } catch (...) {
        // Error handling
        return nullptr;
    }
}

bool OpenDeviceWidget::isValidIpAddress(const std::string& ip) const {
    // Simple regex-like validation for IP address format
    // This is a basic check - a full implementation would be more robust
    if (ip.empty()) return true; // Empty is valid for MSGQ
    
    // Check if it's a valid IPv4 address
    // In a real implementation, you'd use a proper IP validation library
    int num_dots = 0;
    for (char c : ip) {
        if (c == '.') num_dots++;
        else if (c < '0' || c > '9') {
            if (c != ' ') continue; // Allow spaces temporarily for logic
            return false;
        }
    }
    
    if (num_dots != 3) return false;
    
    // Further validation would be needed for a complete implementation
    // For now, just check basic format
    return true;
}