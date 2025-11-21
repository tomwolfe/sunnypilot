#include "tools/cabana/streams/replaystream.h"

#include <memory>
#include <thread>
#include <chrono>

#include "tools/replay/replay.h"
#include "tools/replay/logreader.h"
#include "tools/replay/util.h"

ReplayStream::ReplayStream(std::shared_ptr<Replay> replay)
    : replay_(replay) {
  // Initialize the base class
  // Connect to replay events

  // Set up event filter to capture CAN messages
  if (replay_) {
    replay_->installEventFilter([this](const tools::replay::Event *event) {
      return this->eventFilter(event);
    });

    // Set up callbacks
    replay_->onSeeking = [this](double seconds) {
      seeking_signal.emit(seconds);
    };

    replay_->onSeekedTo = [this](double seconds) {
      seekedTo_signal.emit(seconds);
      // Update current time
      current_sec_ = seconds;
    };

    replay_->onQLogLoaded = [this](std::shared_ptr<tools::replay::LogReader> logreader) {
      // Handle qlog loading if needed
    };
  }
}

ReplayStream::~ReplayStream() {
  if (replay_) {
    replay_->pause(true);
  }
}

void ReplayStream::start() {
  if (replay_) {
    replay_->start();
    // Update sources when replay starts
    sources = {0}; // Set source to 0 for replay data
    sourcesUpdated_signal.emit(sources);
  }
}

std::string ReplayStream::routeName() const {
  if (replay_) {
    return replay_->route().getName();
  }
  return "No route loaded";
}

std::string ReplayStream::carFingerprint() const {
  if (replay_) {
    return replay_->carFingerprint();
  }
  return "";
}

void ReplayStream::seekTo(double ts) {
  if (replay_) {
    replay_->seekTo(ts, false);
  }
}

double ReplayStream::minSeconds() const {
  if (replay_) {
    return replay_->minSeconds();
  }
  return 0.0;
}

double ReplayStream::maxSeconds() const {
  if (replay_) {
    return replay_->maxSeconds();
  }
  return 0.0;
}

bool ReplayStream::isPaused() const {
  if (replay_) {
    return replay_->isPaused();
  }
  return true;
}

void ReplayStream::pause(bool pause) {
  if (replay_) {
    replay_->pause(pause);
    if (pause) {
      paused_signal.emit();
    } else {
      resume_signal.emit();
    }
  }
}

double ReplayStream::getSpeed() {
  if (replay_) {
    return replay_->getSpeed();
  }
  return 1.0;
}

void ReplayStream::setSpeed(float speed) {
  if (replay_) {
    replay_->setSpeed(speed);
  }
}

bool ReplayStream::eventFilter(const tools::replay::Event *event) {
  // Process CAN messages specifically
  if (event && event->which == cereal::Event::Which::CAN) {
    // Parse the capnp event to extract CAN data
    capnp::FlatArrayMessageReader reader(event->data);
    auto event_reader = reader.getRoot<cereal::Event>();

    const auto &can_events = event_reader.getCan();
    std::set<MessageId> new_message_ids;
    bool has_new_ids = false;

    for (const auto &can_event : can_events) {
      uint8_t src = can_event.getSrc();
      uint32_t address = can_event.getAddress();
      auto dat = can_event.getDat();

      MessageId msg_id(src, address);
      new_message_ids.insert(msg_id);

      // Update the event in the stream
      uint64_t mono_time = event->mono_time;
      std::lock_guard lk(mutex_);
      const CanEvent *can_event_ptr = newEvent(mono_time, can_event);
      if (can_event_ptr) {
        all_events_.push_back(can_event_ptr);
        events_[msg_id].push_back(can_event_ptr);

        // Update the last message
        updateEvent(msg_id, (mono_time - beginMonoTime()) / 1e9,
                   dat.begin(), dat.size());

        // Track the highest/lowest timestamps for time range
        if (mono_time > lastest_event_ts) lastest_event_ts = mono_time;
        if (begin_event_ts == 0 || mono_time < begin_event_ts) begin_event_ts = mono_time;
      }
    }

    // Check if we have new message IDs
    for (const auto& msg_id : new_message_ids) {
      if (new_msgs_.insert(msg_id).second) {
        has_new_ids = true;
      }
    }

    if (has_new_ids) {
      // Emit the message received signal
      msgsReceived_signal.emit(&new_message_ids, has_new_ids);
    }
  }

  // Process other event types if needed
  return true;
}