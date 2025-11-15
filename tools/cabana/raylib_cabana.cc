#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include "tools/cabana/raylib_mainwin.h"
#include "tools/cabana/streams/devicestream.h"
#include "tools/cabana/streams/pandastream.h"
#include "tools/cabana/streams/replaystream.h"
#include "tools/cabana/streams/socketcanstream.h"

int main(int argc, char *argv[]) {
    std::cout << "Starting Cabana with Raylib UI..." << std::endl;

    // Initialize AbstractStream pointer (this would normally come from command line parsing)
    AbstractStream *stream = nullptr;

    // Parse command line arguments similar to the original implementation
    std::string route;
    std::string dbc_file;
    std::vector<std::string> args;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--demo" || arg == "-demo") {
            // Use demo route
            route = "demo_route";  // This would be replaced with actual demo route
        } else if (arg == "--dbc" && i + 1 < argc) {
            dbc_file = argv[++i];
        } else if (arg == "--panda" || arg == "-panda") {
            try {
                stream = new PandaStream(nullptr);  // Simplified
            } catch (const std::exception& e) {
                std::cerr << "Error creating Panda stream: " << e.what() << std::endl;
                return 1;
            }
        } else if (arg[0] != '-') {
            // Positional argument - route
            route = arg;
        }
        // Add other argument parsing as needed
    }

    // Create stream if not already created
    if (!stream) {
        if (route.empty()) {
            // No route specified, use dummy stream
            stream = new DummyStream(nullptr);
        } else {
            // Create replay stream for the route
            // This would be a more complex implementation in reality
            stream = new DummyStream(nullptr);  // Placeholder
        }
    }

    try {
        // Create and run the Raylib-based main window
        // Note: The constructor may need to be adapted for the new implementation
        // For now, we'll call a simplified initialization
        CabanaMainWindow *main_window = new CabanaMainWindow();

        // Run the application
        main_window->run();

        delete main_window;
    } catch (const std::exception& e) {
        std::cerr << "Error running Cabana: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Cabana closed." << std::endl;
    return 0;
}