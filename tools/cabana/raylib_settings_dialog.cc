#include "raylib_settings_dialog.h"
#include "utils/util.h"
#include "tools/cabana/utils/filebrowserdialog.h"
#include <iostream>

// Global instance
SettingsDlg* settingsDlg = nullptr;

SettingsDlg::SettingsDlg(void* parent) {
  // Initialize window bounds
  windowBounds = {400, 200, 500, 500};  // x, y, width, height
  
  // Initialize UI element bounds
  themeDropdownBounds = {windowBounds.x + 20, windowBounds.y + 40, 200, 30};
  fpsSliderBounds = {windowBounds.x + 20, windowBounds.y + 100, 300, 20};
  cachedMinutesSliderBounds = {windowBounds.x + 20, windowBounds.y + 160, 300, 20};
  chartHeightSliderBounds = {windowBounds.x + 20, windowBounds.y + 220, 300, 20};
  dragDirectionDropdownBounds = {windowBounds.x + 20, windowBounds.y + 280, 200, 30};
  logLivestreamCheckboxBounds = {windowBounds.x + 20, windowBounds.y + 340, 20, 20};
  logPathTextBounds = {windowBounds.x + 20, windowBounds.y + 380, 300, 30};
  browseButtonBounds = {windowBounds.x + 330, windowBounds.y + 380, 100, 30};
  okButtonBounds = {windowBounds.x + 100, windowBounds.y + 440, 100, 40};
  cancelButtonBounds = {windowBounds.x + 250, windowBounds.y + 440, 100, 40};
  
  // Load current settings
  themeSelection = settings.theme;
  fpsValue = settings.fps;
  cachedMinutesValue = settings.max_cached_minutes;
  chartHeightValue = settings.chart_height;
  dragDirectionValue = static_cast<int>(settings.drag_direction);
  logLivestreamEnabled = settings.log_livestream;
  logPath = settings.log_path;
}

SettingsDlg::~SettingsDlg() {
}

void SettingsDlg::show() {
  visible = true;
}

void SettingsDlg::hide() {
  visible = false;
}

void SettingsDlg::update() {
  if (!visible) return;

  Vector2 mousePoint = GetMousePosition();
  
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    // Check theme dropdown
    if (CheckCollisionPointRec(mousePoint, themeDropdownBounds)) {
      themeSelection = (themeSelection + 1) % 3;  // 0=Auto, 1=Light, 2=Dark
    }
    // Check drag direction dropdown
    else if (CheckCollisionPointRec(mousePoint, dragDirectionDropdownBounds)) {
      dragDirectionValue = (dragDirectionValue + 1) % 4;  // 0=MSB First, 1=LSB First, 2=Always LE, 3=Always BE
    }
    // Check log livestream checkbox
    else if (CheckCollisionPointRec(mousePoint, logLivestreamCheckboxBounds)) {
      logLivestreamEnabled = !logLivestreamEnabled;
    }
    // Check browse button
    else if (CheckCollisionPointRec(mousePoint, browseButtonBounds)) {
      // Show directory browser dialog
      if (fileBrowserDialog == nullptr) {
        fileBrowserDialog = new FileBrowserDialog();
      }

      std::string selected_path = fileBrowserDialog->browseDirectory("Select Log Directory", logPath);
      if (!selected_path.empty()) {
        logPath = selected_path;
      }
    }
    // Check OK button
    else if (CheckCollisionPointRec(mousePoint, okButtonBounds)) {
      save();
      hide();
    }
    // Check Cancel button
    else if (CheckCollisionPointRec(mousePoint, cancelButtonBounds)) {
      hide();
    }
    // Check sliders
    else if (CheckCollisionPointRec(mousePoint, fpsSliderBounds)) {
      activeSlider = 0;
      isDragging = true;
    }
    else if (CheckCollisionPointRec(mousePoint, cachedMinutesSliderBounds)) {
      activeSlider = 1;
      isDragging = true;
    }
    else if (CheckCollisionPointRec(mousePoint, chartHeightSliderBounds)) {
      activeSlider = 2;
      isDragging = true;
    }
  }
  
  // Handle dragging for sliders
  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && isDragging && activeSlider != -1) {
    float sliderValue = (mousePoint.x - (windowBounds.x + 20)) / 300.0f;  // Normalize to 0-1
    
    switch (activeSlider) {
      case 0: // FPS slider
        fpsValue = 10 + (int)(sliderValue * 90);  // Range 10-100
        if (fpsValue < 10) fpsValue = 10;
        if (fpsValue > 100) fpsValue = 100;
        // Snap to nearest 10
        fpsValue = ((fpsValue + 5) / 10) * 10;
        break;
      case 1: // Cached minutes slider
        cachedMinutesValue = 30 + (int)(sliderValue * 90);  // Range 30-120
        if (cachedMinutesValue < 30) cachedMinutesValue = 30;
        if (cachedMinutesValue > 120) cachedMinutesValue = 120;
        break;
      case 2: // Chart height slider
        chartHeightValue = 100 + (int)(sliderValue * 400);  // Range 100-500
        if (chartHeightValue < 100) chartHeightValue = 100;
        if (chartHeightValue > 500) chartHeightValue = 500;
        // Snap to nearest 10
        chartHeightValue = ((chartHeightValue + 5) / 10) * 10;
        break;
    }
  } else if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
    isDragging = false;
    activeSlider = -1;
  }
}

void SettingsDlg::draw() {
  if (!visible) return;
  
  // Draw window background
  DrawRectangleRec(windowBounds, Color{240, 240, 240, 255}); // Light gray
  DrawRectangleLines(windowBounds.x, windowBounds.y, windowBounds.width, windowBounds.height, BLACK);
  
  // Draw window title
  DrawText("Settings", windowBounds.x + 10, windowBounds.y + 10, 20, BLACK);
  
  // Draw theme label and dropdown
  DrawText("Color Theme:", themeDropdownBounds.x, themeDropdownBounds.y - 20, 14, BLACK);
  DrawRectangleRec(themeDropdownBounds, LIGHTGRAY);
  DrawRectangleLines(themeDropdownBounds.x, themeDropdownBounds.y, themeDropdownBounds.width, themeDropdownBounds.height, BLACK);
  const char* themes[] = {"Automatic", "Light", "Dark"};
  DrawText(themes[themeSelection], themeDropdownBounds.x + 5, themeDropdownBounds.y + 5, 12, BLACK);
  
  // Draw FPS slider
  DrawText("FPS:", fpsSliderBounds.x, fpsSliderBounds.y - 20, 14, BLACK);
  DrawRectangleRec(fpsSliderBounds, LIGHTGRAY);
  float fpsSliderPos = ((float)(fpsValue - 10) / 90.0f) * 300.0f; // Convert FPS value to position
  Rectangle sliderKnob = {fpsSliderBounds.x + fpsSliderPos - 5, fpsSliderBounds.y - 5, 10, 30};
  DrawRectangleRec(sliderKnob, DARKGRAY);
  DrawText(TextFormat("%d", fpsValue), fpsSliderBounds.x + 310, fpsSliderBounds.y - 15, 12, BLACK);
  
  // Draw cached minutes slider
  DrawText("Max Cached Minutes:", cachedMinutesSliderBounds.x, cachedMinutesSliderBounds.y - 20, 14, BLACK);
  DrawRectangleRec(cachedMinutesSliderBounds, LIGHTGRAY);
  float cachedMinutesSliderPos = ((float)(cachedMinutesValue - 30) / 90.0f) * 300.0f; // Convert to position
  Rectangle cachedMinutesSliderKnob = {cachedMinutesSliderBounds.x + cachedMinutesSliderPos - 5, cachedMinutesSliderBounds.y - 5, 10, 30};
  DrawRectangleRec(cachedMinutesSliderKnob, DARKGRAY);
  DrawText(TextFormat("%d", cachedMinutesValue), cachedMinutesSliderBounds.x + 310, cachedMinutesSliderBounds.y - 15, 12, BLACK);
  
  // Draw chart height slider
  DrawText("Chart Height:", chartHeightSliderBounds.x, chartHeightSliderBounds.y - 20, 14, BLACK);
  DrawRectangleRec(chartHeightSliderBounds, LIGHTGRAY);
  float chartHeightSliderPos = ((float)(chartHeightValue - 100) / 400.0f) * 300.0f; // Convert to position
  Rectangle chartHeightSliderKnob = {chartHeightSliderBounds.x + chartHeightSliderPos - 5, chartHeightSliderBounds.y - 5, 10, 30};
  DrawRectangleRec(chartHeightSliderKnob, DARKGRAY);
  DrawText(TextFormat("%d", chartHeightValue), chartHeightSliderBounds.x + 310, chartHeightSliderBounds.y - 15, 12, BLACK);
  
  // Draw drag direction label and dropdown
  DrawText("Drag Direction:", dragDirectionDropdownBounds.x, dragDirectionDropdownBounds.y - 20, 14, BLACK);
  DrawRectangleRec(dragDirectionDropdownBounds, LIGHTGRAY);
  DrawRectangleLines(dragDirectionDropdownBounds.x, dragDirectionDropdownBounds.y, dragDirectionDropdownBounds.width, dragDirectionDropdownBounds.height, BLACK);
  const char* dragDirections[] = {"MSB First", "LSB First", "Always LE", "Always BE"};
  DrawText(dragDirections[dragDirectionValue], dragDirectionDropdownBounds.x + 5, dragDirectionDropdownBounds.y + 5, 12, BLACK);
  
  // Draw log livestream checkbox and label
  DrawRectangleRec(logLivestreamCheckboxBounds, WHITE);
  DrawRectangleLines(logLivestreamCheckboxBounds.x, logLivestreamCheckboxBounds.y, logLivestreamCheckboxBounds.width, logLivestreamCheckboxBounds.height, BLACK);
  if (logLivestreamEnabled) {
    DrawRectangle(logLivestreamCheckboxBounds.x + 4, logLivestreamCheckboxBounds.y + 4, 
                  logLivestreamCheckboxBounds.width - 8, logLivestreamCheckboxBounds.height - 8, BLACK);
  }
  DrawText("Enable live stream logging", logLivestreamCheckboxBounds.x + 30, logLivestreamCheckboxBounds.y - 2, 14, BLACK);
  
  // Draw log path label, text box and browse button
  DrawText("Log File Location:", logPathTextBounds.x, logPathTextBounds.y - 20, 14, BLACK);
  DrawRectangleRec(logPathTextBounds, WHITE);
  DrawRectangleLines(logPathTextBounds.x, logPathTextBounds.y, logPathTextBounds.width, logPathTextBounds.height, BLACK);
  DrawText(TextFormat("%.30s%s", logPath.c_str(), logPath.length() > 30 ? "..." : ""), 
           logPathTextBounds.x + 5, logPathTextBounds.y + 5, 10, BLACK);
  DrawRectangleRec(browseButtonBounds, GRAY);
  DrawRectangleLines(browseButtonBounds.x, browseButtonBounds.y, browseButtonBounds.width, browseButtonBounds.height, BLACK);
  DrawText("Browse...", browseButtonBounds.x + 10, browseButtonBounds.y + 8, 12, BLACK);
  
  // Draw OK and Cancel buttons
  DrawRectangleRec(okButtonBounds, GREEN);
  DrawRectangleLines(okButtonBounds.x, okButtonBounds.y, okButtonBounds.width, okButtonBounds.height, BLACK);
  DrawText("OK", okButtonBounds.x + 35, okButtonBounds.y + 12, 16, WHITE);
  
  DrawRectangleRec(cancelButtonBounds, RED);
  DrawRectangleLines(cancelButtonBounds.x, cancelButtonBounds.y, cancelButtonBounds.width, cancelButtonBounds.height, BLACK);
  DrawText("Cancel", cancelButtonBounds.x + 25, cancelButtonBounds.y + 12, 16, WHITE);
}

void SettingsDlg::save() {
  settings.theme = themeSelection;
  settings.fps = fpsValue;
  settings.max_cached_minutes = cachedMinutesValue;
  settings.chart_height = chartHeightValue;
  settings.drag_direction = static_cast<Settings::DragDirection>(dragDirectionValue);
  settings.log_livestream = logLivestreamEnabled;
  settings.log_path = logPath;
  
  // Trigger theme change if needed
  if (themeSelection != 0) {  // If not automatic
    // In a real implementation, this would call utils::setTheme(settings.theme)
    // For now we just store the value
  }
}