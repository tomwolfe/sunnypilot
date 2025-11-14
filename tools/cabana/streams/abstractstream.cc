#include "tools/cabana/streams/abstractstream.h"

AbstractStream::AbstractStream() : AbstractStreamBase() {
  // Initialize any additional members if needed
}

DummyStream::DummyStream() : AbstractStream() {
  // Initialize the dummy stream
}

AbstractStream *can = nullptr;