#pragma once

#include "raylib.h"
#include "raylib_settings.h"
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