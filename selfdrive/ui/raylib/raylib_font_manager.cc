#include "raylib_font_manager.h"
#include "common/util.h"
#include <fstream>
#include <streambuf>

FontManager& FontManager::getInstance() {
    static FontManager instance;
    return instance;
}

void FontManager::initialize() {
    if (initialized) return;
    
    // Load default font sizes we'll need
    for (int size : {12, 16, 24, 30, 32, 36, 48, 64, 72, 84, 96, 120}) {
        getFont(size);
    }
    
    initialized = true;
}

Font FontManager::getFont(int size) {
    auto it = fonts.find(size);
    if (it != fonts.end()) {
        return it->second;
    }
    
    Font font = loadFont(size);
    fonts[size] = font;
    return font;
}

Font FontManager::loadFont(int size) {
    // Try to load from embedded resources first, then fall back to system default
    // In a real implementation, we'd load from the assets directory
    
    // For now, return the default font at the requested size if possible
    // In a full implementation, we would load the Inter font family as used in the Qt version
    Font defaultFont = GetFontDefault();
    
    // Create a font image at the requested size
    // Note: In a real implementation, we would load the actual Inter font files
    
    return defaultFont;
}

void FontManager::unloadAllFonts() {
    for (auto& pair : fonts) {
        // Note: In Raylib, we typically only unload custom fonts, not the default font
        // For custom loaded fonts we would call UnloadFont(pair.second)
    }
    fonts.clear();
    initialized = false;
}

// Enhanced text rendering functions
void drawTextFitted(const std::string& text, Rectangle bounds, float fontSize, const Color& color) {
    FontManager& fm = FontManager::getInstance();
    Font font = fm.getFont((int)fontSize);
    
    // Calculate text width to determine if we need to scale
    float textWidth = MeasureTextEx(font, text.c_str(), fontSize, 0).x;
    
    if (textWidth > bounds.width) {
        // Scale down to fit
        float scale = bounds.width / textWidth;
        fontSize *= scale;
        font = fm.getFont((int)fontSize);
    }
    
    Vector2 position = {bounds.x, bounds.y + (bounds.height - fontSize) / 2};
    DrawTextEx(font, text.c_str(), position, fontSize, 0, color);
}

void drawTextCentered(const std::string& text, float x, float y, float width, float height, float fontSize, const Color& color) {
    FontManager& fm = FontManager::getInstance();
    Font font = fm.getFont((int)fontSize);
    
    Vector2 size = MeasureTextEx(font, text.c_str(), fontSize, 0);
    Vector2 position = {
        x + (width - size.x) / 2,
        y + (height - size.y) / 2
    };
    
    DrawTextEx(font, text.c_str(), position, fontSize, 0, color);
}