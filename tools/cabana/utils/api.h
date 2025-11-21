#pragma once

#include <string>
#include <functional>

// Minimal API interface for Raylib-based cabana
namespace api {

class HttpRequest {
public:
    virtual ~HttpRequest() = default;
    virtual void send(const std::string &url) = 0;
};

// Simple HTTP request function for fetching data
void get(const std::string &url, std::function<void(const std::string&, bool)> callback);

} // namespace api