#pragma once

#include <functional>
#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <typeindex>
#include <typeinfo>

// Base event class for all events in the system
struct BaseEvent {
  virtual ~BaseEvent() = default;
  virtual std::string getType() const = 0;
};

// Event manager for Raylib-based cabana UI
class EventManager {
public:
  static EventManager& instance();
  
  // Subscribe to an event type with a callback
  template<typename EventType>
  uint64_t subscribe(std::function<void(const EventType&)> callback);
  
  // Unsubscribe from an event
  void unsubscribe(uint64_t subscription_id);
  
  // Publish an event to all subscribers of that event type
  template<typename EventType>
  void publish(const EventType& event);
  
  // Clear all subscriptions (useful for cleanup)
  void clear();

private:
  EventManager() = default;
  ~EventManager() = default;
  EventManager(const EventManager&) = delete;
  EventManager& operator=(const EventManager&) = delete;
  
  struct SubscriptionBase {
    virtual ~SubscriptionBase() = default;
    virtual void execute(const BaseEvent& event) = 0;
  };
  
  template<typename EventType>
  struct Subscription : public SubscriptionBase {
    std::function<void(const EventType&)> callback;
    Subscription(std::function<void(const EventType&)> cb) : callback(std::move(cb)) {}
    
    void execute(const BaseEvent& event) override {
      if (const auto* typed_event = dynamic_cast<const EventType*>(&event)) {
        callback(*typed_event);
      }
    }
  };
  
  std::mutex mutex_;
  std::unordered_map<std::type_index, std::unordered_map<uint64_t, std::unique_ptr<SubscriptionBase>>> subscriptions_;
  std::atomic<uint64_t> next_id_{1};
};

// Specific event types for cabana

struct CanMessageReceivedEvent : public BaseEvent {
  uint8_t source;
  uint32_t address;
  std::vector<uint8_t> data;
  double timestamp;
  
  CanMessageReceivedEvent(uint8_t src, uint32_t addr, const std::vector<uint8_t>& dat, double ts)
    : source(src), address(addr), data(dat), timestamp(ts) {}
  
  std::string getType() const override { return "CanMessageReceived"; }
};

struct StreamStartedEvent : public BaseEvent {
  std::string stream_name;
  
  StreamStartedEvent(const std::string& name) : stream_name(name) {}
  
  std::string getType() const override { return "StreamStarted"; }
};

struct RouteLoadedEvent : public BaseEvent {
  std::string route_name;
  std::string car_fingerprint;
  double min_seconds;
  double max_seconds;
  
  RouteLoadedEvent(const std::string& route, const std::string& car_f, double min_s, double max_s)
    : route_name(route), car_fingerprint(car_f), min_seconds(min_s), max_seconds(max_s) {}
  
  std::string getType() const override { return "RouteLoaded"; }
};

struct SeekEvent : public BaseEvent {
  double target_time;
  
  SeekEvent(double time) : target_time(time) {}
  
  std::string getType() const override { return "Seek"; }
};

struct DBCFileLoadedEvent : public BaseEvent {
  std::string file_path;
  
  DBCFileLoadedEvent(const std::string& path) : file_path(path) {}
  
  std::string getType() const override { return "DBCFileLoaded"; }
};

// Template implementations
template<typename EventType>
uint64_t EventManager::subscribe(std::function<void(const EventType&)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t id = next_id_++;
  subscriptions_[std::type_index(typeid(EventType))][id] = 
    std::make_unique<Subscription<EventType>>(std::move(callback));
  return id;
}

template<typename EventType>
void EventManager::publish(const EventType& event) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto type_it = subscriptions_.find(std::type_index(typeid(EventType)));
  if (type_it != subscriptions_.end()) {
    // Work on a copy of the subscriptions to avoid issues during callbacks
    auto subscriptions_copy = type_it->second;
    for (const auto& [id, subscription] : subscriptions_copy) {
      subscription->execute(event);
    }
  }
}

inline EventManager& EventManager::instance() {
  static EventManager instance;
  return instance;
}