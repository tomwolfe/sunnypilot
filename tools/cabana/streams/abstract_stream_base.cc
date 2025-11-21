#include "tools/cabana/streams/abstract_stream_base.h"
#include "tools/cabana/utils/util.h"
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstring>

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
  // Implementation to suppress signals based on DBC definitions
  // This would iterate through defined signals and apply suppression
}

void AbstractStreamBase::mergeEvents(const std::vector<const CanEvent *> &events) {
  std::lock_guard lk(mutex_);
  for (const CanEvent *event : events) {
    MessageId id(event->src, event->address);
    all_events_.push_back(event);
    events_[id].push_back(event);

    // Track time range
    if (event->mono_time > lastest_event_ts) lastest_event_ts = event->mono_time;
    if (begin_event_ts == 0 || event->mono_time < begin_event_ts) begin_event_ts = event->mono_time;
  }

  eventsMerged_signal.emit(events_);
}

const CanEvent *AbstractStreamBase::newEvent(uint64_t mono_time, const cereal::CanData::Reader &c) {
  auto dat = c.getDat();
  size_t can_event_size = sizeof(CanEvent) + dat.size();

  if (!event_buffer_) event_buffer_ = std::make_unique<MonotonicBuffer>(1024 * 1024); // 1MB buffer

  CanEvent *event = (CanEvent*)event_buffer_->allocate(can_event_size);
  if (event) {
    event->mono_time = mono_time;
    event->src = c.getSrc();
    event->address = c.getAddress();
    event->size = dat.size();
    memcpy(event->dat, dat.begin(), dat.size());
  }
  return event;
}

void AbstractStreamBase::updateEvent(const MessageId &id, double sec, const uint8_t *data, uint8_t size) {
  std::lock_guard lk(mutex_);

  // Initialize message if not exists
  CanData &can_data = messages_[id];
  can_data.compute(id, data, size, sec, getSpeed(), masks_[id]);

  // Update last message
  last_msgs[id] = can_data;

  updateLastMsgsTo(sec);
  updateMasks();
}

void AbstractStreamBase::waitForSeekFinished() {
  std::unique_lock lk(mutex_);
  seek_finished_cv_.wait(lk, [this] { return seek_finished_; });
  seek_finished_ = false;
}

void AbstractStreamBase::updateLastMessages() {
  std::lock_guard lk(mutex_);
  last_msgs = messages_;
}

void AbstractStreamBase::updateLastMsgsTo(double sec) {
  current_sec_ = sec;
  std::lock_guard lk(mutex_);
  messages_.clear();

  // Find events up to this time and update message data
  for (auto &[msg_id, events] : events_) {
    auto it = std::upper_bound(events.begin(), events.end(), sec * 1e9 + beginMonoTime(), CompareCanEvent{});

    if (it != events.begin()) {
      --it;
      const CanEvent *event = *it;
      if (event) {
        CanData &can_data = messages_[msg_id];
        can_data.compute(msg_id, event->dat, event->size,
                        (event->mono_time - beginMonoTime()) / 1e9,
                        getSpeed(), masks_[msg_id]);
      }
    }
  }

  // Update last messages
  updateLastMessages();
}

void AbstractStreamBase::updateMasks() {
  // Update masks based on current message data
  // This would be used to highlight specific bits in the binary view
}

void CanData::compute(const MessageId &msg_id, const uint8_t *data, const int size, double current_sec,
                      double playback_speed, const std::vector<uint8_t> &mask, double in_freq) {
  if (dat.size() != size) {
    dat.resize(size);
    colors.resize(size);
    last_changes.resize(size);
    bit_flip_counts.resize(size);
  }

  // Copy data
  memcpy(dat.data(), data, size);

  // Calculate frequency
  double delta_time = current_sec - ts;
  if (delta_time > 0) {
    freq = 1.0 / delta_time;
  }

  // Update bit flip counts
  for (int i = 0; i < size && i < dat.size(); ++i) {
    if (dat[i] != this->dat[i]) {
      // Track which bits changed
      uint8_t changed_bits = dat[i] ^ this->dat[i];
      for (int bit = 0; bit < 8; ++bit) {
        if (changed_bits & (1 << bit)) {
          bit_flip_counts[i][bit]++;
        }
      }
    }

    // Update colors based on data changes
    if (dat[i] != this->dat[i]) {
      colors[i] = SimpleColor(1.0f, 0.0f, 0.0f, 1.0f); // Red for changed
    } else {
      colors[i] = SimpleColor(0.0f, 0.0f, 0.0f, 1.0f); // Black for unchanged
    }
  }

  ts = current_sec;
  count++;
}

AbstractStreamBase *can_base = nullptr;