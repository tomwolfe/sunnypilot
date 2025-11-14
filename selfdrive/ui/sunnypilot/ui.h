/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/raylib/raylib_ui_state_full.h"

// Define SunnyLink models with basic structures (to be updated with full definitions later)
enum class RoleType {
  Admin,
  Sponsor,
  User
};

enum class SponsorTier {
  Free,
  Tier1,
  Tier2,
  Tier3
};

struct RoleModel {
  RoleType roleType;

  template<typename T>
  T as() const {
    // Basic casting functionality
    return T();
  }
};

struct SponsorRoleModel {
  RoleType roleType;
  SponsorTier roleTier;

  SponsorRoleModel(RoleType type, SponsorTier tier) : roleType(type), roleTier(tier) {}
};

struct UserModel {
  std::string user_id;
};

enum OnroadTimerStatusToggle {
  NONE,
  PAUSE,
  RESUME
};

// Since UIState is now the Raylib version, UIStateSP extends that
class UIStateSP : public UIState {
public:
  UIStateSP();
  void updateStatus() override;
  inline bool engaged() const override {
    return scene.started && (
      (*sm)["selfdriveState"].getSelfdriveState().getEnabled() || (*sm)["selfdriveStateSP"].getSelfdriveStateSP().getMads().getEnabled()
    );
  }
  void setSunnylinkRoles(const std::vector<RoleModel> &roles);
  void setSunnylinkDeviceUsers(const std::vector<UserModel> &users);

  inline std::vector<RoleModel> sunnylinkDeviceRoles() const { return sunnylinkRoles; }
  inline bool isSunnylinkAdmin() const {
    return std::any_of(sunnylinkRoles.begin(), sunnylinkRoles.end(), [](const RoleModel &role) {
      return role.roleType == RoleType::Admin;
    });
  }
  inline bool isSunnylinkSponsor() const {
    return std::any_of(sunnylinkRoles.begin(), sunnylinkRoles.end(), [](const RoleModel &role) {
      return role.roleType == RoleType::Sponsor && role.as<SponsorRoleModel>().roleTier != SponsorTier::Free;
    });
  }
  inline SponsorRoleModel sunnylinkSponsorRole() const {
    std::optional<SponsorRoleModel> sponsorRoleWithHighestTier = std::nullopt;
    for (const auto &role : sunnylinkRoles) {
      if(role.roleType != RoleType::Sponsor)
        continue;

      if (auto sponsorRole = role.as<SponsorRoleModel>(); !sponsorRoleWithHighestTier.has_value() || sponsorRoleWithHighestTier->roleTier < sponsorRole.roleTier) {
        sponsorRoleWithHighestTier = sponsorRole;
      }
    }
    return sponsorRoleWithHighestTier.value_or(SponsorRoleModel(RoleType::Sponsor, SponsorTier::Free));
  }
  inline SponsorTier sunnylinkSponsorTier() const {
    return sunnylinkSponsorRole().roleTier;
  }
  inline std::vector<UserModel> sunnylinkDeviceUsers() const { return sunnylinkUsers; }
  inline bool isSunnylinkPaired() const {
    return std::any_of(sunnylinkUsers.begin(), sunnylinkUsers.end(), [](const UserModel &user) {
      return user.user_id.toLower() != "unregisteredsponsor" && user.user_id.toLower() != "temporarysponsor";
    });
  }
  void reset_onroad_sleep_timer(OnroadTimerStatusToggle toggleTimerStatus = OnroadTimerStatusToggle::NONE);

  // Callbacks for events (replacing Qt signals)
  std::function<void(bool)> sunnylinkRoleChangedCallback;
  std::function<void(std::vector<RoleModel>)> sunnylinkRolesChangedCallback;
  std::function<void(std::vector<UserModel>)> sunnylinkDeviceUsersChangedCallback;
  std::function<void(const UIStateSP&)> uiUpdateCallbackSP;

private:
  std::vector<RoleModel> sunnylinkRoles = {};
  std::vector<UserModel> sunnylinkUsers = {};
  void* param_watcher; // Replace with appropriate non-Qt type
};

UIStateSP *uiStateSP();
inline UIStateSP *uiState() { return uiStateSP(); };

// device management class
class DeviceSP : public Device {
public:
  DeviceSP();

private:
  Params params;
  void handleDisplayPowerChanged(bool on);
};

DeviceSP *deviceSP();
inline DeviceSP *device() { return deviceSP(); }

void ui_update_params_sp(UIStateSP *s);
