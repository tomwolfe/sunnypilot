#pragma once

#include <raylib.h>
#include <string>
#include <unordered_map>

// Font manager to handle font loading and caching
class FontManager {
public:
    static FontManager& getInstance();
    
    // Initialize font manager and load default fonts
    void initialize();
    
    // Get a font with specified size (loads if not cached)
    Font getFont(int size = 48);
    
    // Unload all fonts
    void unloadAllFonts();

private:
    FontManager() = default;
    ~FontManager() = default;
    
    std::unordered_map<int, Font> fonts;
    bool initialized = false;
    
    // Load a specific font size from resources
    Font loadFont(int size);
};

// Enhanced text rendering functions
void drawTextFitted(const std::string& text, Rectangle bounds, float fontSize, const Color& color);
void drawTextCentered(const std::string& text, float x, float y, float width, float height, float fontSize, const Color& color);