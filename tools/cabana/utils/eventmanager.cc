#include "tools/cabana/utils/eventmanager.h"

void EventManager::unsubscribe(uint64_t subscription_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& [type, subscriptions] : subscriptions_) {
    subscriptions.erase(subscription_id);
  }
}

void EventManager::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  subscriptions_.clear();
}