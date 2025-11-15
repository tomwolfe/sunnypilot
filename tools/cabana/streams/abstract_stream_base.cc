#include "tools/cabana/streams/abstract_stream_base.h"
#include <algorithm>
#include <chrono>
#include <thread>

AbstractStreamBase::AbstractStreamBase() = default;

void AbstractStreamBase::setTimeRange(const std::optional<std::pair<double, double>> &range) {
  if (time_range_ != range) {
    time_range_ = range;
    timeRangeChanged_signal.emit(range);
  }
}

bool AbstractStreamBase::isMessageActive(const MessageId &id) const {
  auto it = last_msgs.find(id);
  return it != last_msgs.end();
}

const CanData &AbstractStreamBase::lastMessage(const MessageId &id) const {
  static CanData empty;
  auto it = last_msgs.find(id);
  return it != last_msgs.end() ? it->second : empty;
}

const std::vector<const CanEvent *> &AbstractStreamBase::events(const MessageId &id) const {
  static std::vector<const CanEvent *> empty;
  auto it = events_.find(id);
  return it != events_.end() ? it->second : empty;
}

std::pair<CanEventIter, CanEventIter> AbstractStreamBase::eventsInRange(const MessageId &id, std::optional<std::pair<double, double>> time_range) const {
  const auto &events = this->events(id);
  if (!time_range) {
    return {events.begin(), events.end()};
  } else {
    const uint64_t start_ts = beginMonoTime() + time_range->first * 1e9;
    const uint64_t end_ts = beginMonoTime() + time_range->second * 1e9;
    auto start_it = std::lower_bound(events.begin(), events.end(), start_ts, CompareCanEvent{});
    auto end_it = std::upper_bound(events.begin(), events.end(), end_ts, CompareCanEvent{});
    return {start_it, end_it};
  }
}

size_t AbstractStreamBase::suppressHighlighted() {
  size_t count = 0;
  for (auto &[id, data] : last_msgs) {
    for (auto &last_change : data.last_changes) {
      if (last_change.suppressed) {
        last_change.suppressed = false;
        ++count;
      }
    }
  }
  return count;
}

void AbstractStreamBase::clearSuppressed() {
  for (auto &[id, data] : last_msgs) {
    for (auto &last_change : data.last_changes) {
      last_change.suppressed = false;
    }
  }
}

void AbstractStreamBase::suppressDefinedSignals(bool suppress) {
  // Implementation would go here
}

void AbstractStreamBase::mergeEvents(const std::vector<const CanEvent *> &events) {
  // Implementation would go here
}

const CanEvent *AbstractStreamBase::newEvent(uint64_t mono_time, const cereal::CanData::Reader &c) {
  // Implementation would go here
  return nullptr;
}

void AbstractStreamBase::updateEvent(const MessageId &id, double sec, const uint8_t *data, uint8_t size) {
  // Implementation would go here
}

void AbstractStreamBase::waitForSeekFinished() {
  std::unique_lock lk(mutex_);
  seek_finished_cv_.wait(lk, [this] { return seek_finished_; });
  seek_finished_ = false;
}

void AbstractStreamBase::updateLastMessages() {
  // Implementation would go here
}

void AbstractStreamBase::updateLastMsgsTo(double sec) {
  // Implementation would go here
}

void AbstractStreamBase::updateMasks() {
  // Implementation would go here
}

void CanData::compute(const MessageId &msg_id, const uint8_t *data, const int size, double current_sec,
                      double playback_speed, const std::vector<uint8_t> &mask, double in_freq) {
  // Implementation would go here
}

AbstractStreamBase *can_base = nullptr;