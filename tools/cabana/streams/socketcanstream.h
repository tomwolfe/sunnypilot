#pragma once

#include <memory>
#include <vector>
#include <string>

#include "tools/cabana/streams/livestream.h"

// Configuration for SocketCAN stream
struct SocketCanStreamConfig {
    std::string device = "";
    bool canFdEnabled = true;
};

class SocketCanStream : public LiveStream {
public:
  SocketCanStream(SocketCanStreamConfig config_ = {});
  ~SocketCanStream() { stop(); }
  inline std::string routeName() const override {
    return "Live Streaming From Socket CAN " + config_.device;
  }

  // Static method to check if SocketCAN is available
  static bool available();

protected:
  bool connect();
  void streamThread() override;

  SocketCanStreamConfig config_ = {};
};

// Widget interface for SocketCAN
class OpenSocketCanInterface {
public:
  OpenSocketCanInterface();
  AbstractStream* open();

private:
  void refreshDevices();
  void buildConfigForm();

  // Configuration for SocketCAN
  SocketCanStreamConfig config_ = {};
};