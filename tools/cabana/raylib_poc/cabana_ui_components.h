#pragma once
#include <raylib.h>
#include <memory>
#include <functional>
#include <string>

// Base UI element class for cabana Raylib implementation
class CabanaUIElement {
public:
  CabanaUIElement(float x, float y, float width, float height);
  virtual ~CabanaUIElement() = default;
  
  virtual void update() = 0;
  virtual void render() = 0;
  virtual void handleInput();
  
  void setPosition(float x, float y);
  void setSize(float width, float height);
  Rectangle getBounds() const { return bounds; }
  void setVisible(bool visible) { isVisible = visible; }
  bool getVisible() const { return isVisible; }

protected:
  Rectangle bounds;
  bool isVisible;
};

class CabanaMainWindow {
public:
  CabanaMainWindow();
  ~CabanaMainWindow();
  void update();
  void render();
  void handleInput();

private:
  std::unique_ptr<CabanaUIElement> messagesPanel;
  std::unique_ptr<CabanaUIElement> videoPanel;
  std::unique_ptr<CabanaUIElement> chartsPanel;
  std::unique_ptr<CabanaUIElement> menuBar;
  std::unique_ptr<CabanaUIElement> statusBar;
  
  bool running = true;
};

// Basic rendering utilities
void drawBackground(Color color = RAYWHITE);
void drawRect(const Rectangle& rect, Color color);
void drawText(const char* text, float x, float y, int fontSize, Color color);
void drawTextCentered(const char* text, Rectangle container, int fontSize, Color color);