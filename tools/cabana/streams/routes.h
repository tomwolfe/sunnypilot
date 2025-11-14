#pragma once

#include <string>
#include <memory>

#include "tools/lib/api.hpp"

class OneShotHttpRequest;

class RoutesDialog {
public:
  RoutesDialog();
  std::string route();

protected:
  void parseDeviceList(const std::string &json, bool success);
  void parseRouteList(const std::string &json, bool success);
  void fetchRoutes();

  std::string selected_route_;
  std::unique_ptr<OneShotHttpRequest> route_requester_;
};
