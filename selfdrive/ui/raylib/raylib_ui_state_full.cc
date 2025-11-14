#include "selfdrive/ui/raylib/raylib_ui_state_full.h"

#include <algorithm>
#include <cmath>

#include "common/transformations/orientation.hpp"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/watchdog.h"
#include "system/hardware/hw.h"
#include "common/params.h"

// Import Eigen for matrix operations (from the original header)
#include <eigen3/Eigen/Dense>

// Define the calibration matrices (from the original header)
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

const UIColor bg_colors [] = {
  [STATUS_DISENGAGED] = UIColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_OVERRIDE] = UIColor(0x91, 0x9b, 0x95, 0xf1),
  [STATUS_ENGAGED] = UIColor(0x17, 0x86, 0x44, 0xf1),
  [STATUS_LAT_ONLY] = UIColor(0x00, 0xc8, 0xc8, 0xf1),
  [STATUS_LONG_ONLY] = UIColor(0x96, 0x1C, 0xA8, 0xf1),
};

void update_sockets_raylib(UIState *s) {
  s->sm->update(0);
}

void update_state_raylib(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  if (sm.updated("liveCalibration")) {
    auto list2rot = [](const capnp::List<float>::Reader &rpy_list) ->Eigen::Matrix3f {
      return euler2rot({rpy_list[0], rpy_list[1], rpy_list[2]}).cast<float>();
    };

    auto live_calib = sm["liveCalibration"].getLiveCalibration();
    if (live_calib.getCalStatus() == cereal::LiveCalibrationData::Status::CALIBRATED) {
      auto device_from_calib = list2rot(live_calib.getRpyCalib());
      auto wide_from_device = list2rot(live_calib.getWideFromDeviceEuler());
      // Update scene calibration matrices
      for (int i = 0; i < 9; i++) {
        s->scene.view_from_calib[i] = (VIEW_FROM_DEVICE * device_from_calib).data()[i];
        s->scene.view_from_wide_calib[i] = (VIEW_FROM_DEVICE * wide_from_device * device_from_calib).data()[i];
      }
    } else {
      for (int i = 0; i < 9; i++) {
        s->scene.view_from_calib[i] = VIEW_FROM_DEVICE.data()[i];
        s->scene.view_from_wide_calib[i] = VIEW_FROM_DEVICE.data()[i];
      }
    }
  }
  
  if (sm.updated("pandaStates")) {
    auto pandaStates = sm["pandaStates"].getPandaStates();
    if (pandaStates.size() > 0) {
      scene.pandaType = pandaStates[0].getPandaType();

      if (scene.pandaType != cereal::PandaState::PandaType::UNKNOWN) {
        scene.ignition = false;
        for (const auto& pandaState : pandaStates) {
          scene.ignition |= pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
        }
      }
    }
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaStates")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  
  if (sm.updated("wideRoadCameraState")) {
    auto cam_state = sm["wideRoadCameraState"].getWideRoadCameraState();
    scene.light_sensor = std::max(100.0f - cam_state.getExposureValPercent(), 0.0f);
  } else if (!sm.allAliveAndValid({"wideRoadCameraState"})) {
    scene.light_sensor = -1;
  }
  
  scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;
  
  auto params = Params();
  scene.is_metric = params.getBool("IsMetric");
  
  // Update camera textures if available
  s->updateRoadCameraTexture();
  s->updateWideRoadCameraTexture();
  s->updateDriverCameraTexture();
}

void ui_update_params_raylib(UIState *s) {
  auto params = Params();
  s->scene.is_metric = params.getBool("IsMetric");
}

void UIState::updateStatus() {
  if (scene.started && (sm->updated("selfdriveState") || sm->updated("selfdriveStateSP"))) {
    auto ss = (*sm)["selfdriveState"].getSelfdriveState();
    auto mads = (*sm)["selfdriveStateSP"].getSelfdriveStateSP().getMads();
    auto state = ss.getState();
    auto state_mads = mads.getState();
    if (state == cereal::SelfdriveState::OpenpilotState::PRE_ENABLED || state == cereal::SelfdriveState::OpenpilotState::OVERRIDING ||
        state_mads == cereal::ModularAssistiveDrivingSystem::ModularAssistiveDrivingSystemState::PAUSED ||
        state_mads == cereal::ModularAssistiveDrivingSystem::ModularAssistiveDrivingSystemState::OVERRIDING) {
      status = STATUS_OVERRIDE;
    } else {
      if (mads.getAvailable()) {
        if (mads.getEnabled() && ss.getEnabled()) {
          status = STATUS_ENGAGED;
        } else if (mads.getEnabled()) {
          status = STATUS_LAT_ONLY;
        } else if (ss.getEnabled()) {
          status = STATUS_LONG_ONLY;
        } else {
          status = STATUS_DISENGAGED;
        }
      } else {
        status = ss.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
      }
    }
  }

  bool engaged_current = engaged();
  if (engaged_current != engaged_prev) {
    engaged_prev = engaged_current;
    if (engagedChangedCallback) {
      engagedChangedCallback(engaged_current);
    }
  }

  // Handle onroad/offroad transition
  if (scene.started != started_prev || sm->frame == 1) {
    if (scene.started) {
      status = STATUS_DISENGAGED;
      scene.started_frame = sm->frame;
    }
    started_prev = scene.started;
    if (offroadTransitionCallback) {
      offroadTransitionCallback(!scene.started);
    }
  }
}

// Update camera textures from messaging data
void UIState::updateRoadCameraTexture() {
  // In a real implementation, this would extract image data from the roadCameraState message
  // and convert it to a Raylib texture
  // For now, this is a placeholder
}

void UIState::updateWideRoadCameraTexture() {
  // In a real implementation, this would extract image data from the wideRoadCameraState message
  // and convert it to a Raylib texture
  // For now, this is a placeholder
}

void UIState::updateDriverCameraTexture() {
  // In a real implementation, this would extract image data from the driverCameraState message
  // and convert it to a Raylib texture
  // For now, this is a placeholder
}

UIState::UIState() {
  sm = std::make_unique<SubMaster>(std::vector<const char*>{
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState",
    "pandaStates", "carParams", "driverMonitoringState", "carState", "driverStateV2",
    "wideRoadCameraState", "roadCameraState", "driverCameraState", "managerState", 
    "selfdriveState", "longitudinalPlan",
  });
  prime_state = new PrimeState();
  auto params = Params();
  language = params.get("LanguageSetting");

  // Initialize timer values
  last_update_time = GetTime();
  update_interval = 1.0f / UI_FREQ;
}

void UIState::update() {
  float current_time = GetTime();
  if (current_time - last_update_time >= update_interval) {
    update_sockets_raylib(this);
    update_state_raylib(this);
    updateStatus();
    
    if (sm->frame % UI_FREQ == 0) {
      watchdog_kick(nanos_since_boot());
    }
    
    if (uiUpdateCallback) {
      uiUpdateCallback(*this);
    }
    
    last_update_time = current_time;
  }
}

Device::Device() : brightness_filter(BACKLIGHT_OFFROAD, 10.0, 0.05) {
  setAwake(true);
  resetInteractiveTimeout();
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);
}

void Device::setAwake(bool on) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    if (displayPowerChangedCallback) {
      displayPowerChangedCallback(awake);
    }
  }
}

void Device::resetInteractiveTimeout(int timeout) {
  auto params = Params();
  int customTimeout = std::stoi(params.get("InteractivityTimeout"));
  if (timeout == -1) {
    timeout = customTimeout == 0 ? (ignition_on ? 10 : 30) : customTimeout;
  }
  interactive_timeout = timeout * UI_FREQ;
}

void Device::updateBrightness(const UIState &s) {
  int brightness;
  auto params = Params();
  int brightness_override = std::stoi(params.get("Brightness"));
  float clipped_brightness = offroad_brightness;
  if (s.scene.started && s.scene.light_sensor >= 0) {
    clipped_brightness = s.scene.light_sensor;

    // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    if (brightness_override == 1) {
      clipped_brightness = std::clamp(100.0f * clipped_brightness, 1.0f, 100.0f);  // Scale back to 1% to 100%
    } else if (brightness_override == 0) {
      clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);  // Scale back to 10% to 100%
    }
  }

  if (brightness_override == 0 || brightness_override == 1) {
    brightness = brightness_filter.update(clipped_brightness);
  } else {
    brightness = brightness_override;
  }

  if (!awake) {
    brightness = 0;
  }

  // Onroad Brightness Control
#ifdef SUNNYPILOT
  if (awake && s.scene.started && s.scene.onroadScreenOffControl && s.scene.onroadScreenOffTimer == 0) {
    brightness = s.scene.onroadScreenOffBrightness * 0.01 * brightness;
  }
#endif

  if (brightness != last_brightness) {
    // For now, just update internally. Hardware::set_brightness would be called in a real implementation
    last_brightness = brightness;
  }
}

void Device::updateWakefulness(const UIState &s) {
  bool ignition_just_turned_off = !s.scene.ignition && ignition_on;
  ignition_on = s.scene.ignition;

  if (ignition_just_turned_off) {
    resetInteractiveTimeout();
  } else if (interactive_timeout > 0 && --interactive_timeout == 0) {
    if (interactiveTimeoutCallback) {
      interactiveTimeoutCallback();
    }
  }

  setAwake(s.scene.ignition || interactive_timeout > 0);
}

#ifndef SUNNYPILOT
UIState *uiState_raylib() {
  static UIState ui_state;
  return &ui_state;
}

Device *device_raylib() {
  static Device _device;
  return &_device;
}
#endif

// Implementation of UI state functionality for Raylib
UIState *uiState() {
  return uiState_raylib();
}

Device *device() {
  return device_raylib();
}