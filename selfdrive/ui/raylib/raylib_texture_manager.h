#pragma once

#include <raylib.h>
#include <string>
#include <unordered_map>

// Texture manager to handle texture loading and caching
class TextureManager {
public:
    static TextureManager& getInstance();
    
    // Load and cache a texture from file path
    Texture2D getTexture(const std::string& path);
    
    // Load and cache a texture from memory data
    Texture2D getTextureFromMemory(const std::string& name, const unsigned char* data, unsigned int size);
    
    // Unload all cached textures
    void unloadAllTextures();

private:
    TextureManager() = default;
    ~TextureManager() = default;
    
    std::unordered_map<std::string, Texture2D> textures;
    bool initialized = false;
};

// Additional UI utilities
float easeInOutCubic(float t);
Rectangle animateRectangle(const Rectangle& start, const Rectangle& end, float progress);