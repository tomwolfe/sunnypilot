#pragma once

// Define OPENPILOT_RAYLIB before including raylib to prevent enum conflicts
#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"

// Temporarily undefine conflicting macros to avoid capnp conflicts
#ifdef RED
#undef RED
#endif
#ifdef GREEN
#undef GREEN
#endif
#ifdef YELLOW
#undef YELLOW
#endif
#ifdef BLUE
#undef BLUE
#endif
#ifdef MAGENTA
#undef MAGENTA
#endif
#ifdef CYAN
#undef CYAN
#endif
#ifdef WHITE
#undef WHITE
#endif
#ifdef BLACK
#undef BLACK
#endif
#ifdef GRAY
#undef GRAY
#endif

#include "raylib_settings.h"

// Define color constants for use in the file after undefining problematic macros
#ifndef RED
#define RED ((Color){230, 41, 55, 255})
#endif
#ifndef GREEN
#define GREEN ((Color){0, 228, 48, 255})
#endif
#ifndef YELLOW
#define YELLOW ((Color){253, 249, 0, 255})
#endif
#ifndef BLUE
#define BLUE ((Color){0, 121, 241, 255})
#endif
#ifndef MAGENTA
#define MAGENTA ((Color){255, 0, 255, 255})
#endif
#ifndef CYAN
#define CYAN ((Color){0, 255, 255, 255})
#endif
#ifndef WHITE
#define WHITE ((Color){255, 255, 255, 255})
#endif
#ifndef BLACK
#define BLACK ((Color){0, 0, 0, 255})
#endif
#ifndef GRAY
#define GRAY ((Color){129, 129, 129, 255})
#endif
#ifndef LIGHTGRAY
#define LIGHTGRAY ((Color){200, 200, 200, 255})
#endif
#ifndef DARKGRAY
#define DARKGRAY ((Color){80, 80, 80, 255})
#endif

#include <string>
#include <vector>

// Raylib-based settings dialog
class SettingsDlg {
public:
  SettingsDlg(void* parent = nullptr);
  ~SettingsDlg();
  void show();
  void hide();
  bool isVisible() const { return visible; }
  void update();
  void draw();
  void save();
  
private:
  bool visible = false;
  Rectangle windowBounds;
  
  // UI elements state
  int themeSelection = 0;
  int fpsValue = 10;
  int cachedMinutesValue = 30;
  int chartHeightValue = 200;
  int dragDirectionValue = 0;
  bool logLivestreamEnabled = true;
  std::string logPath;
  
  // UI controls
  Rectangle themeDropdownBounds;
  Rectangle fpsSliderBounds;
  Rectangle cachedMinutesSliderBounds;
  Rectangle chartHeightSliderBounds;
  Rectangle dragDirectionDropdownBounds;
  Rectangle logLivestreamCheckboxBounds;
  Rectangle logPathTextBounds;
  Rectangle browseButtonBounds;
  Rectangle okButtonBounds;
  Rectangle cancelButtonBounds;
  
  // Temporary state
  bool isDragging = false;
  int activeSlider = -1; // -1 = none, 0 = fps, 1 = cached minutes, 2 = chart height
};

// Create a global instance
extern SettingsDlg* settingsDlg;