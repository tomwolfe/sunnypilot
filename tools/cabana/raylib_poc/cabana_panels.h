#pragma once
#include "cabana_ui_components.h"
#include <vector>
#include <string>

// Replacement for MessagesWidget
class MessagesPanel : public CabanaUIElement {
public:
  MessagesPanel(float x, float y, float width, float height);
  void update() override;
  void render() override;

private:
  std::vector<std::string> messages;
  int selectedMessageIndex = -1;
  float scrollOffset = 0.0f;
};

// Replacement for VideoWidget 
class VideoPanel : public CabanaUIElement {
public:
  VideoPanel(float x, float y, float width, float height);
  void update() override;
  void render() override;

private:
  // Video playback state would be managed here
  bool isPlaying = false;
  float currentTime = 0.0f;
  float totalTime = 0.0f;
};

// Replacement for ChartsWidget
class ChartsPanel : public CabanaUIElement {
public:
  ChartsPanel(float x, float y, float width, float height);
  void update() override;
  void render() override;

private:
  // Chart data and rendering would be managed here
};

// Replacement for menu bar
class MenuBar : public CabanaUIElement {
public:
  MenuBar(float x, float y, float width, float height);
  void update() override;
  void render() override;

private:
  // Menu items and state management
};