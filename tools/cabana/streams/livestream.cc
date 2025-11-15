#include "tools/cabana/streams/livestream.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>

#include "common/timing.h"
#include "common/util.h"

struct LiveStream::Logger {
  Logger() : start_ts(seconds_since_epoch()), segment_num(-1) {}

  void write(kj::ArrayPtr<capnp::word> data) {
    int n = (seconds_since_epoch() - start_ts) / 60.0;
    if (std::exchange(segment_num, n) != segment_num) {
      // Format using standard C++
      auto time_t = start_ts;
      std::tm* tm_info = std::gmtime(&time_t);
      char buffer[100];
      std::strftime(buffer, sizeof(buffer), "%Y-%m-%d--%H-%M-%S", tm_info);
      std::string time_str(buffer);
      std::string dir = settings.log_path + "/" + time_str + "--" + std::to_string(n);
      util::create_directories(dir, 0755);
      fs.reset(new std::ofstream((dir + "/rlog").c_str(), std::ios::binary | std::ios::out));
    }

    auto bytes = data.asBytes();
    fs->write((const char*)bytes.begin(), bytes.size());
  }

  std::unique_ptr<std::ofstream> fs;
  int segment_num;
  uint64_t start_ts;
};

LiveStream::LiveStream() : AbstractStream() {
  if (settings.log_livestream) {
    logger = std::make_unique<Logger>();
  }

  // Note: We use callback functions in the Raylib implementation
}

LiveStream::~LiveStream() {
  stop();
}

void LiveStream::startUpdateTimer() {
  // Update time tracking for Raylib implementation
  last_update_time = std::chrono::steady_clock::now();
}

void LiveStream::start() {
  stream_thread = std::make_unique<std::thread>([=]() { streamThread(); });
  startUpdateTimer();
  begin_date_time = SimpleDateTime::currentDateTime();
}

void LiveStream::stop() {
  if (!stream_thread) return;

  stream_thread->join(); // Wait for thread to finish instead of interrupting
  stream_thread.reset();
}

// called in streamThread
void LiveStream::handleEvent(kj::ArrayPtr<capnp::word> data) {
  if (logger) {
    logger->write(data);
  }

  capnp::FlatArrayMessageReader reader(data);
  auto event = reader.getRoot<cereal::Event>();
  if (event.which() == cereal::Event::Which::CAN) {
    const uint64_t mono_time = event.getLogMonoTime();
    std::lock_guard lk(lock);
    for (const auto &c : event.getCan()) {
      received_events_.push_back(newEvent(mono_time, c));
    }
  }
}

void LiveStream::updateEvents() {
  static double prev_speed = 1.0;

  if (first_update_ts == 0) {
    first_update_ts = nanos_since_boot();
    first_event_ts = current_event_ts = all_events_.back()->mono_time;
  }

  if (paused_ || prev_speed != speed_) {
    prev_speed = speed_;
    first_update_ts = nanos_since_boot();
    first_event_ts = current_event_ts;
    return;
  }

  uint64_t last_ts = post_last_event && speed_ == 1.0
                       ? all_events_.back()->mono_time
                       : first_event_ts + (nanos_since_boot() - first_update_ts) * speed_;
  auto first = std::upper_bound(all_events_.cbegin(), all_events_.cend(), current_event_ts, CompareCanEvent());
  auto last = std::upper_bound(first, all_events_.cend(), last_ts, CompareCanEvent());

  for (auto it = first; it != last; ++it) {
    const CanEvent *e = *it;
    MessageId id = {.source = e->src, .address = e->address};
    updateEvent(id, (e->mono_time - begin_event_ts) / 1e9, e->dat, e->size);
    current_event_ts = e->mono_time;
  }

  // Call our custom event handlers
  privateUpdateLastMsgsSignal.emit();
}

void LiveStream::seekTo(double sec) {
  sec = std::max(0.0, sec);
  first_update_ts = nanos_since_boot();
  current_event_ts = first_event_ts = std::min<uint64_t>(sec * 1e9 + begin_event_ts, lastest_event_ts);
  post_last_event = (first_event_ts == lastest_event_ts);
  seekedTo_signal.emit((current_event_ts - begin_event_ts) / 1e9);
}

void LiveStream::pause(bool pause) {
  paused_ = pause;
  if (pause) {
    paused_signal.emit();
  } else {
    resume_signal.emit();
  }
}
