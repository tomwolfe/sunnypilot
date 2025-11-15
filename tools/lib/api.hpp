#pragma once

#include <string>
#include <functional>
#include <memory>

// Minimal API header for HTTP requests in Raylib-based UI
namespace api {

class HttpRequest {
public:
    virtual ~HttpRequest() = default;
    virtual void send(const std::string &url) = 0;
    std::function<void(const std::string&, bool)> done_callback;
};

} // namespace api