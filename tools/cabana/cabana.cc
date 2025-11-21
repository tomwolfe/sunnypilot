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
#include "tools/cabana/utils/logger.h"

int main(int argc, char *argv[]) {
  // Initialize logging
  auto& logger = Logger::instance();
  logger.setLogLevel(LogLevel::INFO);
  logger.info("MAIN", "Starting Cabana with Raylib UI...");

  // Initialize AbstractStream pointer (this would normally come from command line parsing)
  AbstractStream *stream = nullptr;
  std::string dbc_file;

  // Parse command line arguments similar to the original implementation
  std::string route;
  std::vector<std::string> allow;
  std::vector<std::string> block;
  std::string data_dir;
  uint32_t flags = 0;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--demo" || arg == "-demo") {
      logger.info("MAIN", "Using demo route: ", DEMO_ROUTE);
      route = DEMO_ROUTE;
    } else if (arg == "--dbc" && i + 1 < argc) {
      dbc_file = argv[++i];
      logger.info("MAIN", "DBC file specified: ", dbc_file);
    } else if (arg == "--panda" || arg == "-panda") {
      try {
        logger.info("MAIN", "Creating Panda stream...");
        PandaStreamConfig config = {};
        stream = new PandaStream(config);  // Default config
        logger.info("MAIN", "Panda stream created successfully");
      } catch (const std::exception& e) {
        logger.error("MAIN", "Error creating Panda stream: ", e.what());
        return 1;
      }
    } else if (arg == "--msgq" || arg == "-msgq") {
      logger.info("MAIN", "Using msgq stream (implementation needed)");
      // For msgq, we might connect to a running openpilot instance
      // Implementation needed
    } else if ((arg == "--zmq" || arg == "-zmq") && i + 1 < argc) {
      try {
        logger.info("MAIN", "Creating ZMQ stream...");
        // Create DeviceStream with ZMQ connection
        // Implementation needed
        i++; // Skip the IP address
        logger.info("MAIN", "ZMQ stream created (placeholder)");
      } catch (const std::exception& e) {
        logger.error("MAIN", "Error creating ZMQ stream: ", e.what());
        return 1;
      }
    } else if ((arg == "--panda-serial" || arg == "-panda-serial") && i + 1 < argc) {
      try {
        PandaStreamConfig config = {};
        config.serial = argv[++i];
        logger.info("MAIN", "Creating Panda stream with serial: ", config.serial);
        stream = new PandaStream(config);
        logger.info("MAIN", "Panda stream with serial created successfully");
      } catch (std::exception &e) {
        logger.error("MAIN", "Error creating Panda stream: ", e.what());
        return 0;
      }
    } else if ((arg == "--socketcan" || arg == "-socketcan") && i + 1 < argc) {
      if (SocketCanStream::available()) {
        SocketCanStreamConfig config = {};
        config.device = argv[++i];
        logger.info("MAIN", "Creating SocketCAN stream with device: ", config.device);
        stream = new SocketCanStream(config);
        logger.info("MAIN", "SocketCAN stream created successfully");
      } else {
        logger.error("MAIN", "SocketCAN is not available");
      }
    } else if (arg == "--auto" || arg == "-auto") {
      logger.info("MAIN", "Auto loading route...");
      // Auto load route - would need more implementation
      route = "auto";
    } else if (arg == "--ecam" || arg == "-ecam") {
      flags |= REPLAY_FLAG_ECAM;
      logger.info("MAIN", "Wide road camera flag enabled");
    } else if (arg == "--qcam" || arg == "-qcam") {
      flags |= REPLAY_FLAG_QCAMERA;
      logger.info("MAIN", "QCamera flag enabled");
    } else if (arg == "--dcam" || arg == "-dcam") {
      flags |= REPLAY_FLAG_DCAM;
      logger.info("MAIN", "Driver camera flag enabled");
    } else if (arg == "--no-vipc" || arg == "-no-vipc") {
      flags |= REPLAY_FLAG_NO_VIPC;
      logger.info("MAIN", "No VIPC flag enabled");
    } else if ((arg == "--data_dir" || arg == "-data_dir") && i + 1 < argc) {
      data_dir = argv[++i];
      logger.info("MAIN", "Data directory specified: ", data_dir);
    } else if (arg[0] != '-') {
      // Positional argument - route
      route = arg;
      logger.info("MAIN", "Route specified: ", route);
    } else {
      logger.warning("MAIN", "Unknown argument: ", arg);
    }
    // Add other argument parsing as needed
  }

  // Create stream if not already created
  if (!stream) {
    if (route.empty() || route == "auto") {
      logger.info("MAIN", "No route specified, using demo route");
      route = DEMO_ROUTE;
    }

    logger.info("MAIN", "Creating replay stream for route: ", route);
    // Create replay stream for the route
    try {
      std::shared_ptr<tools::replay::Replay> replay = std::make_shared<tools::replay::Replay>(route, allow, block, nullptr, flags, data_dir);
      stream = new ReplayStream(replay);
      logger.info("MAIN", "Replay stream created successfully");
    } catch (const std::exception& e) {
      logger.error("MAIN", "Error creating Replay stream: ", e.what());
      return 1;
    }
  }

  // Set the global stream pointer
  can = stream;
  logger.info("MAIN", "Global stream pointer set");

  try {
    logger.info("MAIN", "Initializing Raylib window...");
    // Initialize Raylib window
    InitWindow(1200, 800, "Cabana - Raylib Edition");
    if (!IsWindowReady()) {
      logger.error("MAIN", "Failed to initialize Raylib window");
      return 1;
    }
    SetTargetFPS(60);
    logger.info("MAIN", "Raylib window initialized successfully");

    // Create and run the Raylib-based main window
    logger.info("MAIN", "Creating main window...");
    CabanaMainWindow main_window;
    logger.info("MAIN", "Main window created, starting application loop");

    // Run the application
    main_window.run();
    logger.info("MAIN", "Application loop finished");

    // Cleanup
    CloseWindow();
    logger.info("MAIN", "Window closed");
  } catch (const std::exception& e) {
    logger.error("MAIN", "Error running Cabana: ", e.what());
    return 1;
  }

  logger.info("MAIN", "Cabana closed.");
  return 0;
}
