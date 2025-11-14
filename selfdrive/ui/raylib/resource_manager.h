#pragma once

#include <vector>
#include <functional>

// Resource manager to handle cleanup of all UI resources
class ResourceManager {
public:
    static ResourceManager& getInstance();
    
    // Add a cleanup function to be called on shutdown
    void addCleanupFunction(std::function<void()> cleanupFunc);
    
    // Run all cleanup functions
    void cleanupAll();
    
    // RAII wrapper for automatic cleanup registration
    class CleanupHandle {
    public:
        CleanupHandle(std::function<void()> cleanupFunc) {
            ResourceManager::getInstance().addCleanupFunction(cleanupFunc);
        }
    };

private:
    ResourceManager() = default;
    std::vector<std::function<void()>> cleanupFunctions;
};

// Initialize and register cleanup at startup
void initializeResources();