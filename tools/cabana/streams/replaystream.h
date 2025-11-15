#pragma once

#include <string>
#include <memory>
#include <vector>

#include "tools/cabana/streams/abstractstream.h"
#include "tools/replay/replay.h"

// ReplayStream class for cabana
class ReplayStream : public AbstractStream {
public:
  ReplayStream(std::shared_ptr<Replay> replay);
  ~ReplayStream() override;

  void start() override;
  std::string routeName() const override;
  std::string carFingerprint() const override;
  void seekTo(double ts) override;

private:
  std::shared_ptr<Replay> replay_;
};