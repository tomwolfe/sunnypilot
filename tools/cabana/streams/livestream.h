#pragma once

#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <chrono>

#include "tools/cabana/streams/abstractstream.h"

class LiveStream : public AbstractStream {
public:
  LiveStream();
  virtual ~LiveStream();
  void start() override;
  void stop();
  inline SimpleDateTime beginDateTime() const { return begin_date_time; }
  inline uint64_t beginMonoTime() const override { return begin_event_ts; }
  double maxSeconds() const override { return std::max(1.0, (lastest_event_ts - begin_event_ts) / 1e9); }
  void setSpeed(float speed) override { speed_ = speed; }
  double getSpeed() override { return speed_; }
  bool isPaused() const override { return paused_; }
  void pause(bool pause) override;
  void seekTo(double sec) override;

protected:
  virtual void streamThread() = 0;
  void handleEvent(kj::ArrayPtr<capnp::word> event);

private:
  void startUpdateTimer();
  void updateEvents();

  std::mutex lock;
  std::unique_ptr<std::thread> stream_thread;
  std::vector<const CanEvent *> received_events_;

  // Using standard timers for Raylib implementation
  std::chrono::steady_clock::time_point last_update_time;

  SimpleDateTime begin_date_time;
  uint64_t begin_event_ts = 0;
  uint64_t lastest_event_ts = 0;
  uint64_t current_event_ts = 0;
  uint64_t first_event_ts = 0;
  uint64_t first_update_ts = 0;
  bool post_last_event = true;
  double speed_ = 1;
  bool paused_ = false;

  struct Logger;
  std::unique_ptr<Logger> logger;
};
