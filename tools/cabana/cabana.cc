#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include "tools/replay/replay.h"  // For DEMO_ROUTE definition
#include "tools/cabana/raylib_mainwin.h"
#include "tools/cabana/raylib_devicestream.h"
#include "tools/cabana/streams/pandastream.h"
#include "tools/cabana/streams/replaystream.h"
#include "tools/cabana/streams/socketcanstream.h"

int main(int argc, char *argv[]) {
  std::cout << "Starting Cabana with Raylib UI..." << std::endl;

  // Initialize AbstractStream pointer (this would normally come from command line parsing)
  AbstractStream *stream = nullptr;
  std::string dbc_file;

  // Parse command line arguments similar to the original implementation
  std::string route;
  std::vector<std::string> args;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--demo" || arg == "-demo") {
      // Use demo route
      route = DEMO_ROUTE;  // This would be replaced with actual demo route
    } else if (arg == "--dbc" && i + 1 < argc) {
      dbc_file = argv[++i];
    } else if (arg == "--panda" || arg == "-panda") {
      try {
        PandaStreamConfig config = {};
        stream = new PandaStream(config);  // Default config
      } catch (const std::exception& e) {
        std::cerr << "Error creating Panda stream: " << e.what() << std::endl;
        return 1;
      }
    } else if (arg == "--msgq" || arg == "-msgq") {
      // Using DummyStream for now - DeviceStream needs to be properly defined
      stream = new DummyStream();
    } else if ((arg == "--zmq" || arg == "-zmq") && i + 1 < argc) {
      try {
        // Create a mock DeviceStream with the IP address
        // For now, using DummyStream since the real DeviceStream might not be available
        stream = new DummyStream();
      } catch (const std::exception& e) {
        std::cerr << "Error creating ZMQ stream: " << e.what() << std::endl;
        return 1;
      }
    } else if ((arg == "--panda-serial" || arg == "-panda-serial") && i + 1 < argc) {
      try {
        PandaStreamConfig config = {};
        config.serial = argv[++i];
        stream = new PandaStream(config);
      } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 0;
      }
    } else if ((arg == "--socketcan" || arg == "-socketcan") && i + 1 < argc) {
      if (SocketCanStream::available()) {
        SocketCanStreamConfig config = {};
        config.device = argv[++i];
        stream = new SocketCanStream(config);
      }
    } else if (arg == "--auto" || arg == "-auto") {
      // Auto load route - would need more implementation
      route = "auto";
    } else if (arg == "--ecam" || arg == "-ecam") {
      // Wide road camera flag - would need to be handled in stream
    } else if (arg == "--qcam" || arg == "-qcam") {
      // Qcamera flag - would need to be handled in stream
    } else if (arg == "--dcam" || arg == "-dcam") {
      // Driver camera flag - would need to be handled in stream
    } else if (arg == "--no-vipc" || arg == "-no-vipc") {
      // No VIPC flag - would need to be handled in stream
    } else if (arg == "--data_dir" || arg == "-data_dir") {
      if (i + 1 < argc) {
        i++; // Skip the directory for now
      }
    } else if (arg[0] != '-') {
      // Positional argument - route
      route = arg;
    }
    // Add other argument parsing as needed
  }

  // Create stream if not already created
  if (!stream) {
    if (route.empty() || route == "auto") {
      // No route specified, show stream selector or use demo
      stream = new DummyStream();
    } else {
      // Create replay stream for the route with default flags
      // Since we don't have a Replay object yet, we'll use DummyStream for now
      stream = new DummyStream();
    }
  }

  try {
    // Initialize Raylib window
    InitWindow(1200, 800, "Cabana - Raylib Edition");
    SetTargetFPS(60);

    // Create and run the Raylib-based main window
    CabanaMainWindow main_window;

    // Run the application
    main_window.run();

    // Cleanup
    CloseWindow();
  } catch (const std::exception& e) {
    std::cerr << "Error running Cabana: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Cabana closed." << std::endl;
  return 0;
}
