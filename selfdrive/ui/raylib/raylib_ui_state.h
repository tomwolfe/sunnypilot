#pragma once

#include <memory>
#include <string>
#include <functional>

#include <raylib.h>

#include "cereal/messaging/messaging.h"
#include "common/mat.h"
#include "common/params.h"
#include "common/util.h"
#include "system/hardware/hw.h"

// Custom color structure for Raylib
struct UIColor {
  unsigned char r, g, b, a;
  UIColor(unsigned char r = 0, unsigned char g = 0, unsigned char b = 0, unsigned char a = 255)
    : r(r), g(g), b(b), a(a) {}

  // Convert to Raylib Color
  Color to_raylib_color() const {
    return (Color){r, g, b, a};
  }
};

// Prime state definitions
enum class PrimeType {
  UNKNOWN = -2,
  UNPAIRED = -1,
  NONE = 0,
  MAGENTA = 1,
  LITE = 2,
  BLUE = 3,
  MAGENTA_NEW = 4,
  PURPLE = 5,
};

struct PrimeState {
  PrimeType prime_type = PrimeType::UNKNOWN;

  // This is a simplified version - in reality, you'd need full implementation
  PrimeType get_type() const { return prime_type; }
  bool is_prime() const {
    return prime_type > PrimeType::NONE;
  }
  void set_type(PrimeType type) { prime_type = type; }
};

const int UI_BORDER_SIZE = 30;
const int UI_HEADER_HEIGHT = 420;

const int UI_FREQ = 20; // Hz
const int BACKLIGHT_OFFROAD = 50;

// Define background colors
extern const UIColor bg_colors[];

typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_OVERRIDE,
  STATUS_ENGAGED,
  STATUS_LAT_ONLY,
  STATUS_LONG_ONLY,
} UIStatus;

const UIColor bg_colors [] = {
  [STATUS_DISENGAGED] = UIColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_OVERRIDE] = UIColor(0x91, 0x9b, 0x95, 0xf1),
  [STATUS_ENGAGED] = UIColor(0x17, 0x86, 0x44, 0xf1),
  [STATUS_LAT_ONLY] = UIColor(0x00, 0xc8, 0xc8, 0xf1),
  [STATUS_LONG_ONLY] = UIColor(0x96, 0x1C, 0xA8, 0xf1),
};

typedef struct UIScene {
  // Placeholder for calibration matrices - will be filled later
  // For now, we'll use the same structure as before
  float view_from_calib[9];
  float view_from_wide_calib[9];
  cereal::PandaState::PandaType pandaType;

  cereal::LongitudinalPersonality personality;

  float light_sensor = -1;
  bool started, ignition, is_metric, recording_audio;
  uint64_t started_frame;
} UIScene;

#ifdef SUNNYPILOT
#include "sunnypilot/ui_scene.h"
#define UIScene UISceneSP
#endif

// UI State class using Raylib
class UIState {
public:
  UIState();
  virtual void updateStatus();
  virtual inline bool engaged() const {
    return scene.started && (*sm)["selfdriveState"].getSelfdriveState().getEnabled();
  }

  std::unique_ptr<SubMaster> sm;
  UIStatus status;
  UIScene scene = {};
  std::string language;
  PrimeState *prime_state;

  // Callback function for UI updates
  std::function<void(const UIState&)> uiUpdateCallback;
  std::function<void(bool)> offroadTransitionCallback;
  std::function<void(bool)> engagedChangedCallback;

protected:
  // Timer functionality using Raylib
  float last_update_time;
  float update_interval;

  void update();
};

#ifndef SUNNYPILOT
UIState *uiState_raylib();
#endif

// Device management class using Raylib
class Device {
public:
  Device();
  bool isAwake() { return awake; }
  void setOffroadBrightness(int brightness) {
    offroad_brightness = std::clamp(brightness, 0, 100);
  }

protected:
  bool awake = false;
  int interactive_timeout = 0;
  bool ignition_on = false;

  int offroad_brightness = BACKLIGHT_OFFROAD;
  int last_brightness = 0;
  FirstOrderFilter brightness_filter;

  void updateBrightness(const UIState &s);
  void updateWakefulness(const UIState &s);
  void setAwake(bool on);

  // Callbacks for device events
  std::function<void(bool)> displayPowerChangedCallback;
  std::function<void()> interactiveTimeoutCallback;

public:
  void resetInteractiveTimeout(int timeout = -1);
  void update(const UIState &s);
};

#ifndef SUNNYPILOT
Device *device_raylib();
#endif

void ui_update_params_raylib(UIState *s);
void update_state_raylib(UIState *s);
void update_sockets_raylib(UIState *s);