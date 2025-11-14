#pragma once

#include <algorithm>
#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>
#include <string>
#include <functional>

#include "cereal/messaging/messaging.h"
#include "tools/cabana/dbc/dbcmanager.h"
#include "tools/cabana/utils/util.h"
#include "tools/replay/util.h"

// Simple color structure to replace QColor
struct SimpleColor {
    float r = 0.0f, g = 0.0f, b = 0.0f, a = 1.0f;
    
    SimpleColor() = default;
    SimpleColor(float r, float g, float b, float a = 1.0f) : r(r), g(g), b(b), a(a) {}
};

// Simple datetime structure to replace QDateTime
struct SimpleDateTime {
    long long timestamp_ms = 0;  // milliseconds since epoch
    
    static SimpleDateTime currentDateTime() {
        return {static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count())};
    }
    
    std::string toString() const {
        // Convert timestamp to readable format
        std::time_t time_t = timestamp_ms / 1000;
        char buffer[100];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&time_t));
        return std::string(buffer);
    }
};

struct CanData {
  void compute(const MessageId &msg_id, const uint8_t *dat, const int size, double current_sec,
               double playback_speed, const std::vector<uint8_t> &mask, double in_freq = 0);

  double ts = 0.;
  uint32_t count = 0;
  double freq = 0;
  std::vector<uint8_t> dat;
  std::vector<SimpleColor> colors;  // Using SimpleColor instead of QColor

  struct ByteLastChange {
    double ts = 0;
    int delta = 0;
    int same_delta_counter = 0;
    bool suppressed = false;
  };
  std::vector<ByteLastChange> last_changes;
  std::vector<std::array<uint32_t, 8>> bit_flip_counts;
  double last_freq_update_ts = 0;
};

struct CanEvent {
  uint8_t src;
  uint32_t address;
  uint64_t mono_time;
  uint8_t size;
  uint8_t dat[];
};

struct CompareCanEvent {
  constexpr bool operator()(const CanEvent *const e, uint64_t ts) const { return e->mono_time < ts; }
  constexpr bool operator()(uint64_t ts, const CanEvent *const e) const { return ts < e->mono_time; }
};

typedef std::unordered_map<MessageId, std::vector<const CanEvent *>> MessageEventsMap;
using CanEventIter = std::vector<const CanEvent *>::const_iterator;

// Simple event system to replace Qt's signals/slots
class EventListener {
public:
    virtual ~EventListener() = default;
    virtual void handleEvent(const std::string& event_type, void* data = nullptr) = 0;
};

template<typename... Args>
class SimpleSignal {
public:
    using Callback = std::function<void(Args...)>;
    
    void connect(Callback callback) {
        callbacks.push_back(std::move(callback));
    }
    
    void emit(Args... args) {
        for (auto& callback : callbacks) {
            callback(args...);
        }
    }
    
private:
    std::vector<Callback> callbacks;
};

// Base class for stream functionality without Qt dependencies
class AbstractStreamBase {
public:
    AbstractStreamBase();
    virtual ~AbstractStreamBase() {}
    virtual void start() = 0;
    virtual bool liveStreaming() const { return true; }
    virtual void seekTo(double ts) {}
    virtual std::string routeName() const = 0;
    virtual std::string carFingerprint() const { return ""; }
    virtual SimpleDateTime beginDateTime() const { return {}; }
    virtual uint64_t beginMonoTime() const { return 0; }
    virtual double minSeconds() const { return 0; }
    virtual double maxSeconds() const { return 0; }
    virtual void setSpeed(float speed) {}
    virtual double getSpeed() { return 1; }
    virtual bool isPaused() const { return false; }
    virtual void pause(bool pause) {}
    void setTimeRange(const std::optional<std::pair<double, double>> &range);
    const std::optional<std::pair<double, double>> &timeRange() const { return time_range_; }

    inline double currentSec() const { return current_sec_; }
    inline uint64_t toMonoTime(double sec) const { return beginMonoTime() + std::max(sec, 0.0) * 1e9; }
    inline double toSeconds(uint64_t mono_time) const { return std::max(0.0, (mono_time - beginMonoTime()) / 1e9); }

    inline const std::unordered_map<MessageId, CanData> &lastMessages() const { return last_msgs; }
    bool isMessageActive(const MessageId &id) const;
    inline const MessageEventsMap &eventsMap() const { return events_; }
    inline const std::vector<const CanEvent *> &allEvents() const { return all_events_; }
    const CanData &lastMessage(const MessageId &id) const;
    const std::vector<const CanEvent *> &events(const MessageId &id) const;
    std::pair<CanEventIter, CanEventIter> eventsInRange(const MessageId &id, std::optional<std::pair<double, double>> time_range) const;

    size_t suppressHighlighted();
    void clearSuppressed();
    void suppressDefinedSignals(bool suppress);

    // Signals as simple callbacks
    SimpleSignal<> paused_signal;
    SimpleSignal<> resume_signal;
    SimpleSignal<double> seeking_signal;
    SimpleSignal<double> seekedTo_signal;
    SimpleSignal<std::optional<std::pair<double, double>>> timeRangeChanged_signal;
    SimpleSignal<const MessageEventsMap&> eventsMerged_signal;
    SimpleSignal<const std::set<MessageId>*, bool> msgsReceived_signal;
    SimpleSignal<const SourceSet&> sourcesUpdated_signal;

public:
    SourceSet sources;

protected:
    void mergeEvents(const std::vector<const CanEvent *> &events);
    const CanEvent *newEvent(uint64_t mono_time, const cereal::CanData::Reader &c);
    void updateEvent(const MessageId &id, double sec, const uint8_t *data, uint8_t size);
    void waitForSeekFinished();
    std::vector<const CanEvent *> all_events_;
    double current_sec_ = 0;
    std::optional<std::pair<double, double>> time_range_;

private:
    void updateLastMessages();
    void updateLastMsgsTo(double sec);
    void updateMasks();

    MessageEventsMap events_;
    std::unordered_map<MessageId, CanData> last_msgs;
    std::unique_ptr<MonotonicBuffer> event_buffer_;

    // Members accessed in multiple threads. (mutex protected)
    std::mutex mutex_;
    std::condition_variable seek_finished_cv_;
    bool seek_finished_ = false;
    std::set<MessageId> new_msgs_;
    std::unordered_map<MessageId, CanData> messages_;
    std::unordered_map<MessageId, std::vector<uint8_t>> masks_;
};

// A global pointer referring to the unique AbstractStream object
extern AbstractStreamBase *can_base;