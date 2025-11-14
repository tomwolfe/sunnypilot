#include "raylib_texture_manager.h"

TextureManager& TextureManager::getInstance() {
    static TextureManager instance;
    return instance;
}

Texture2D TextureManager::getTexture(const std::string& path) {
    auto it = textures.find(path);
    if (it != textures.end()) {
        return it->second;
    }
    
    // Load texture and cache it
    Texture2D texture = LoadTexture(path.c_str());
    textures[path] = texture;
    return texture;
}

Texture2D TextureManager::getTextureFromMemory(const std::string& name, const unsigned char* data, unsigned int size) {
    auto it = textures.find(name);
    if (it != textures.end()) {
        return it->second;
    }
    
    // Create image from memory and convert to texture
    Image image = LoadImageFromMemory(".png", data, size); // assuming PNG, but format could be passed as parameter
    Texture2D texture = LoadTextureFromImage(image);
    UnloadImage(image);
    
    textures[name] = texture;
    return texture;
}

void TextureManager::unloadAllTextures() {
    for (auto& pair : textures) {
        UnloadTexture(pair.second);
    }
    textures.clear();
}

// Additional UI utilities
float easeInOutCubic(float t) {
    return t < 0.5f ? 4 * t * t * t : (t - 1) * (2 * t - 2) * (2 * t - 2) + 1;
}

Rectangle animateRectangle(const Rectangle& start, const Rectangle& end, float progress) {
    float easedProgress = easeInOutCubic(progress);
    return Rectangle{
        start.x + (end.x - start.x) * easedProgress,
        start.y + (end.y - start.y) * easedProgress,
        start.width + (end.width - start.width) * easedProgress,
        start.height + (end.height - start.height) * easedProgress
    };
}