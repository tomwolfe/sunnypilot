#include "selfdrive/ui/ui.h"

#include <algorithm>
#include <cmath>

#include "common/transformations/orientation.hpp"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/watchdog.h"
#include "system/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00

// Raylib implementation
void update_sockets(UIState *s) {
  update_sockets_raylib(s);
}

void update_state(UIState *s) {
  update_state_raylib(s);
}

void ui_update_params(UIState *s) {
  ui_update_params_raylib(s);
}

UIState *uiState() {
  return uiState_raylib();
}

Device *device() {
  return device_raylib();
}