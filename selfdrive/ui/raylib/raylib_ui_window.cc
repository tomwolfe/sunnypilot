#include "selfdrive/ui/raylib/raylib_ui_window.h"
#include "selfdrive/ui/raylib/raylib_ui_state_full.h"
#include "selfdrive/ui/raylib/raylib_ui_components.h"
#include "selfdrive/ui/raylib/raylib_settings_components.h"
#include "selfdrive/ui/raylib/raylib_extra_components.h"
#include "selfdrive/ui/raylib/raylib_font_manager.h"

// Basic UI element implementation
UIElement::UIElement(float x, float y, float width, float height)
  : bounds({x, y, width, height}), isVisible(true) {
}

void UIElement::setPosition(float x, float y) {
  bounds.x = x;
  bounds.y = y;
}

void UIElement::setSize(float width, float height) {
  bounds.width = width;
  bounds.height = height;
}

// Implementation of the main UI window
RaylibMainWindow::RaylibMainWindow() {
  // Initialize UI elements with screen dimensions
  float screen_width = GetScreenWidth();
  float screen_height = GetScreenHeight();

  // Initialize font manager
  FontManager::getInstance().initialize();

  // Create UI components
  offroadHome = std::make_unique<OffroadHomeElement>(0, 0, screen_width, screen_height);
  onroadWindow = std::make_unique<OnroadWindowElement>(0, 0, screen_width, screen_height);
  settingsWindow = std::make_unique<SettingsWindowElement>(0, 0, screen_width, screen_height);
  sidebar = std::make_unique<SidebarElement>(0, 0, 200, screen_height);
  bodyWindow = std::make_unique<BodyWindowElement>(0, 0, screen_width, screen_height);
  driverViewWindow = std::make_unique<DriverViewWindowElement>(0, 0, screen_width, screen_height);

  // Set up the callback for when driver view is closed
  if (driverViewWindow) {
    driverViewWindow->doneCallback = [this]() {
      showDriverView(false);
    };
  }
}

// Update the destructor to cleanup resources
RaylibMainWindow::~RaylibMainWindow() {
  // Unload all fonts and textures when shutting down
  // FontManager::getInstance().unloadAllFonts();
  // This would typically be called when shutting down the app
}

void RaylibMainWindow::update(const UIState &s) {
  // Handle UI state transitions
  if (currentUIState == UIStateType::ONROAD && !s.scene.started) {
    // Transition from onroad to offroad
    currentUIState = UIStateType::OFFROAD_HOME;
  } else if (currentUIState != UIStateType::ONROAD && s.scene.started) {
    // Transition from offroad to onroad
    currentUIState = UIStateType::ONROAD;
  }

  updateActiveUI(s);
}

void RaylibMainWindow::render(const UIState &s) {
  // Draw background based on UI status
  drawBackground(bg_colors[s.status]);

  // Render the active UI
  renderActiveUI();
}

void RaylibMainWindow::openSettings(int index, const std::string &param) {
  currentUIState = UIStateType::SETTINGS;
  if (settingsWindow) {
    settingsWindow->setCurrentPanel(index, param);
  }
}

void RaylibMainWindow::closeSettings() {
  if (currentUIState == UIStateType::SETTINGS) {
    // Return to appropriate view based on whether the car is started
    currentUIState = (uiState_raylib()->scene.started) ? UIStateType::ONROAD : UIStateType::OFFROAD_HOME;
  }
}

void RaylibMainWindow::showDriverView(bool show) {
  if (show) {
    currentUIState = UIStateType::DRIVER_VIEW;
  } else {
    currentUIState = UIStateType::OFFROAD_HOME;
  }
}

void RaylibMainWindow::updateActiveUI(const UIState &s) {
  // Update the currently active UI component
  switch (currentUIState) {
    case UIStateType::OFFROAD_HOME:
      if (offroadHome) offroadHome->update(s);
      break;
    case UIStateType::ONROAD:
      if (onroadWindow) onroadWindow->update(s);
      break;
    case UIStateType::BODY:
      if (bodyWindow) bodyWindow->update(s);
      break;
    case UIStateType::DRIVER_VIEW:
      if (driverViewWindow) driverViewWindow->update(s);
      break;
    case UIStateType::SETTINGS:
      if (settingsWindow) settingsWindow->update(s);
      break;
  }
}

void RaylibMainWindow::renderActiveUI() const {
  // Render the currently active UI component
  switch (currentUIState) {
    case UIStateType::OFFROAD_HOME:
      if (offroadHome && offroadHome->getVisible()) {
        offroadHome->render();
        if (sidebar && sidebar->getVisible()) sidebar->render();
      }
      break;
    case UIStateType::ONROAD:
      if (onroadWindow && onroadWindow->getVisible()) onroadWindow->render();
      break;
    case UIStateType::BODY:
      if (bodyWindow && bodyWindow->getVisible()) bodyWindow->render();
      break;
    case UIStateType::DRIVER_VIEW:
      if (driverViewWindow && driverViewWindow->getVisible()) driverViewWindow->render();
      break;
    case UIStateType::SETTINGS:
      if (settingsWindow && settingsWindow->getVisible()) settingsWindow->render();
      break;
  }
}

// Basic rendering functions
void drawBackground(const UIColor &color) {
  ClearBackground(color.to_raylib_color());
}

void drawRect(const Rectangle &rect, const UIColor &color) {
  DrawRectangleRec(rect, color.to_raylib_color());
}

void drawText(const char* text, float x, float y, int fontSize, const UIColor &color) {
  FontManager& fm = FontManager::getInstance();
  Font font = fm.getFont(fontSize);
  DrawTextEx(font, text, (Vector2){x, y}, fontSize, 0, color.to_raylib_color());
}

void drawImage(const char* imagePath, float x, float y, float width, float height) {
  TextureManager& tm = TextureManager::getInstance();
  Texture2D texture = tm.getTexture(imagePath);

  // Draw texture with specified dimensions
  DrawTexturePro(
    texture,
    (Rectangle){0, 0, (float)texture.width, (float)texture.height},  // Source rectangle
    (Rectangle){x, y, width, height},  // Destination rectangle
    (Vector2){0, 0},  // Origin
    0.0f,  // Rotation
    WHITE  // Tint
  );
}

// Input handling
bool isMouseClicked() {
  return IsMouseButtonPressed(MOUSE_LEFT_BUTTON);
}

Vector2 getMousePosition() {
  return GetMousePosition();
}