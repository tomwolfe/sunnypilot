#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>

#include "cereal/messaging/messaging.h"
#include "common/mat.h"
#include "common/params.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "selfdrive/ui/raylib/raylib_ui_state_full.h"

const int UI_BORDER_SIZE = 30;
const int UI_HEADER_HEIGHT = 420;

const int UI_FREQ = 20; // Hz
const int BACKLIGHT_OFFROAD = 50;

const Eigen::Matrix3f VIEW_FROM_DEVICE = (Eigen::Matrix3f() <<
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0,
  1.0, 0.0, 0.0).finished();

const Eigen::Matrix3f FCAM_INTRINSIC_MATRIX = (Eigen::Matrix3f() <<
  2648.0, 0.0, 1928.0 / 2,
  0.0, 2648.0, 1208.0 / 2,
  0.0, 0.0, 1.0).finished();

// tici ecam focal probably wrong? magnification is not consistent across frame
// Need to retrain model before this can be changed
const Eigen::Matrix3f ECAM_INTRINSIC_MATRIX = (Eigen::Matrix3f() <<
  567.0, 0.0, 1928.0 / 2,
  0.0, 567.0, 1208.0 / 2,
  0.0, 0.0, 1.0).finished();

typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_OVERRIDE,
  STATUS_ENGAGED,
  STATUS_LAT_ONLY,
  STATUS_LONG_ONLY,
} UIStatus;

// Define background colors using Raylib compatible format
#include <raylib.h>
const Color bg_colors [] = {
  [STATUS_DISENGAGED] = (Color){0x17, 0x33, 0x49, 0xc8},
  [STATUS_OVERRIDE] = (Color){0x91, 0x9b, 0x95, 0xf1},
  [STATUS_ENGAGED] = (Color){0x17, 0x86, 0x44, 0xf1},
  [STATUS_LAT_ONLY] = (Color){0x00, 0xc8, 0xc8, 0xf1},
  [STATUS_LONG_ONLY] = (Color){0x96, 0x1C, 0xA8, 0xf1},
};

typedef struct UIScene {
  Eigen::Matrix3f view_from_calib = VIEW_FROM_DEVICE;
  Eigen::Matrix3f view_from_wide_calib = VIEW_FROM_DEVICE;
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

// Just use the Raylib UI state which is defined in raylib_ui_state_full.h

UIState *uiState();
Device *device();

void ui_update_params(UIState *s);
void update_state(UIState *s);
void update_sockets(UIState *s);
