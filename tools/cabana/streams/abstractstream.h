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

// CanData, CanEvent, and CompareCanEvent are defined in abstract_stream_base.h to avoid redefinition
// This file should only include the class definitions that inherit from the base

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
