#pragma once

#include <string>
#include <memory>
#include <functional>

#include "tools/lib/api.hpp"

class RoutesDialog {
public:
  RoutesDialog();
  std::string route();

protected:
  void parseDeviceList(const std::string &json, bool success);
  void parseRouteList(const std::string &json, bool success);
  void fetchRoutes();

  std::string selected_route_;
  std::unique_ptr<api::HttpRequest> route_requester_;
};
