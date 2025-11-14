#include "resource_manager.h"
#include "raylib_font_manager.h"
#include "raylib_texture_manager.h"

ResourceManager& ResourceManager::getInstance() {
    static ResourceManager instance;
    return instance;
}

void ResourceManager::addCleanupFunction(std::function<void()> cleanupFunc) {
    cleanupFunctions.push_back(cleanupFunc);
}

void ResourceManager::cleanupAll() {
    // Run cleanup functions in reverse order
    for (auto it = cleanupFunctions.rbegin(); it != cleanupFunctions.rend(); ++it) {
        (*it)();
    }
    cleanupFunctions.clear();
}

void initializeResources() {
    // Register global cleanup functions
    ResourceManager::getInstance().addCleanupFunction([]() {
        FontManager::getInstance().unloadAllFonts();
    });
}