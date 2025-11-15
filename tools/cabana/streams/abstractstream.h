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

#include "cereal/messaging/messaging.h"
#include "tools/cabana/dbc/dbcmanager.h"
#include "tools/cabana/utils/util.h"
#include "tools/replay/util.h"
#include "tools/cabana/streams/abstract_stream_base.h"  // Include our Raylib-compatible base

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

// AbstractStream now inherits from our Raylib-compatible base
class AbstractStream : public AbstractStreamBase {
public:
  AbstractStream();
  virtual ~AbstractStream() {}
  virtual void start() = 0;
  virtual std::string routeName() const = 0;

  // Methods for Raylib implementation
  virtual std::string carFingerprint() const { return ""; }
  virtual SimpleDateTime beginDateTime() const { return {}; }

public:
  SourceSet sources;
};

// Raylib-compatible version of the stream widget concept
class AbstractOpenStreamInterface {
public:
  virtual ~AbstractOpenStreamInterface() = default;
  virtual AbstractStream* open() = 0;
};

class DummyStream : public AbstractStream {
public:
  DummyStream();
  std::string routeName() const override { return "No Stream"; }
  void start() override {}
};

// A global pointer referring to the unique AbstractStream object
extern AbstractStream *can;
