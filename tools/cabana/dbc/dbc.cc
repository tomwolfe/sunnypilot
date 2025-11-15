#include "tools/cabana/dbc/dbc.h"

#include <algorithm>
#include <functional>
#include <cctype>

// Simple hash function for MessageId since we removed Qt's qHash
size_t hash_combine(size_t seed, size_t h) {
  return seed ^ (h + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}

size_t qHash(const MessageId &item) {
  size_t h1 = std::hash<uint8_t>{}(item.source);
  size_t h2 = std::hash<uint32_t>{}(item.address);
  return hash_combine(h1, h2);
}

// cabana::Msg

cabana::Msg::~Msg() {
  for (auto s : sigs) {
    delete s;
  }
}

cabana::Signal *cabana::Msg::addSignal(const cabana::Signal &sig) {
  auto s = sigs.emplace_back(new cabana::Signal(sig));
  update();
  return s;
}

cabana::Signal *cabana::Msg::updateSignal(const std::string &sig_name, const cabana::Signal &new_sig) {
  auto s = sig(sig_name);
  if (s) {
    *s = new_sig;
    update();
  }
  return s;
}

void cabana::Msg::removeSignal(const std::string &sig_name) {
  auto it = std::find_if(sigs.begin(), sigs.end(), [&](auto &s) { return s->name == sig_name; });
  if (it != sigs.end()) {
    delete *it;
    sigs.erase(it);
    update();
  }
}

cabana::Msg &cabana::Msg::operator=(const cabana::Msg &other) {
  address = other.address;
  name = other.name;
  size = other.size;
  comment = other.comment;
  transmitter = other.transmitter;

  for (auto s : sigs) delete s;
  sigs.clear();
  for (auto s : other.sigs) {
    sigs.push_back(new cabana::Signal(*s));
  }

  update();
  return *this;
}

cabana::Signal *cabana::Msg::sig(const std::string &sig_name) const {
  auto it = std::find_if(sigs.begin(), sigs.end(), [&](auto &s) { return s->name == sig_name; });
  return it != sigs.end() ? *it : nullptr;
}

int cabana::Msg::indexOf(const cabana::Signal *sig) const {
  for (int i = 0; i < sigs.size(); ++i) {
    if (sigs[i] == sig) return i;
  }
  return -1;
}

std::string cabana::Msg::newSignalName() {
  std::string new_name;
  for (int i = 1; /**/; ++i) {
    new_name = "NEW_SIGNAL_" + std::to_string(i);
    if (sig(new_name) == nullptr) break;
  }
  return new_name;
}

void cabana::Msg::update() {
  if (transmitter.empty()) {
    transmitter = DEFAULT_NODE_NAME;
  }
  mask.assign(size, 0x00);
  multiplexor = nullptr;

  // sort signals
  std::sort(sigs.begin(), sigs.end(), [](auto l, auto r) {
    return std::tie(r->type, l->multiplex_value, l->start_bit, l->name) <
           std::tie(l->type, r->multiplex_value, r->start_bit, r->name);
  });

  for (auto sig : sigs) {
    if (sig->type == cabana::Signal::Type::Multiplexor) {
      multiplexor = sig;
    }
    sig->update();

    // update mask
    int i = sig->msb / 8;
    int bits = sig->size;
    while (i >= 0 && i < size && bits > 0) {
      int lsb = (int)(sig->lsb / 8) == i ? sig->lsb : i * 8;
      int msb = (int)(sig->msb / 8) == i ? sig->msb : (i + 1) * 8 - 1;

      int sz = msb - lsb + 1;
      int shift = (lsb - (i * 8));

      mask[i] |= ((1ULL << sz) - 1) << shift;

      bits -= sz;
      i = sig->is_little_endian ? i - 1 : i + 1;
    }
  }

  for (auto sig : sigs) {
    sig->multiplexor = sig->type == cabana::Signal::Type::Multiplexed ? multiplexor : nullptr;
    if (!sig->multiplexor) {
      if (sig->type == cabana::Signal::Type::Multiplexed) {
        sig->type = cabana::Signal::Type::Normal;
      }
      sig->multiplex_value = 0;
    }
  }
}

// cabana::Signal

void cabana::Signal::update() {
  updateMsbLsb(*this);
  if (receiver_name.empty()) {
    receiver_name = DEFAULT_NODE_NAME;
  }

  float h = 19 * (float)lsb / 64.0;
  h = fmod(h, 1.0);
  size_t hash = qHash(name);
  float s = 0.25 + 0.25 * (float)(hash & 0xff) / 255.0;
  float v = 0.75 + 0.25 * (float)((hash >> 8) & 0xff) / 255.0;

  // Convert HSV to RGB color (replacing QColor::fromHsvF)
  float c = v * s;
  float x = c * (1 - fabs(fmod(h * 6, 2) - 1));
  float m = v - c;
  float r, g, b;

  if (0 <= h && h < 1.0/6) { r = c; g = x; b = 0; }
  else if (1.0/6 <= h && h < 2.0/6) { r = x; g = c; b = 0; }
  else if (2.0/6 <= h && h < 3.0/6) { r = 0; g = c; b = x; }
  else if (3.0/6 <= h && h < 4.0/6) { r = 0; g = x; b = c; }
  else if (4.0/6 <= h && h < 5.0/6) { r = x; g = 0; b = c; }
  else { r = c; g = 0; b = x; }

  color = Color((int)((r + m) * 255), (int)((g + m) * 255), (int)((b + m) * 255));
  precision = std::max(num_decimals(factor), num_decimals(offset));
}

std::string cabana::Signal::formatValue(double value, bool with_unit) const {
  // Show enum string
  int64_t raw_value = round((value - offset) / factor);
  for (const auto &[val, desc] : val_desc) {
    if (std::abs(raw_value - val) < 1e-6) {
      return std::string(desc);
    }
  }

  // Format number using standard C++ (replace QString::number)
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  std::string val_str(buffer);

  if (with_unit && !unit.empty()) {
    val_str += " " + unit;
  }
  return val_str;
}

bool cabana::Signal::getValue(const uint8_t *data, size_t data_size, double *val) const {
  if (multiplexor && get_raw_value(data, data_size, *multiplexor) != multiplex_value) {
    return false;
  }
  *val = get_raw_value(data, data_size, *this);
  return true;
}

bool cabana::Signal::operator==(const cabana::Signal &other) const {
  return name == other.name && size == other.size &&
         start_bit == other.start_bit &&
         msb == other.msb && lsb == other.lsb &&
         is_signed == other.is_signed && is_little_endian == other.is_little_endian &&
         factor == other.factor && offset == other.offset &&
         min == other.min && max == other.max && comment == other.comment && unit == other.unit && val_desc == other.val_desc &&
         multiplex_value == other.multiplex_value && type == other.type && receiver_name == other.receiver_name;
}

// helper functions

double get_raw_value(const uint8_t *data, size_t data_size, const cabana::Signal &sig) {
  const int msb_byte = sig.msb / 8;
  if (msb_byte >= (int)data_size) return 0;

  const int lsb_byte = sig.lsb / 8;
  uint64_t val = 0;

  // Fast path: signal fits in a single byte
  if (msb_byte == lsb_byte) {
    val = (data[msb_byte] >> (sig.lsb & 7)) & ((1ULL << sig.size) - 1);
  } else {
    // Multi-byte case: signal spans across multiple bytes
    int bits = sig.size;
    int i = msb_byte;
    const int step = sig.is_little_endian ? -1 : 1;
    while (i >= 0 && i < (int)data_size && bits > 0) {
      const int msb = (i == msb_byte) ? sig.msb & 7 : 7;
      const int lsb = (i == lsb_byte) ? sig.lsb & 7 : 0;
      const int nbits = msb - lsb + 1;
      val = (val << nbits) | ((data[i] >> lsb) & ((1ULL << nbits) - 1));
      bits -= nbits;
      i += step;
    }
  }

  // Sign extension (if needed)
  if (sig.is_signed && (val & (1ULL << (sig.size - 1)))) {
    val |= ~((1ULL << sig.size) - 1);
  }

  return static_cast<int64_t>(val) * sig.factor + sig.offset;
}

void updateMsbLsb(cabana::Signal &s) {
  if (s.is_little_endian) {
    s.lsb = s.start_bit;
    s.msb = s.start_bit + s.size - 1;
  } else {
    s.lsb = flipBitPos(flipBitPos(s.start_bit) + s.size - 1);
    s.msb = s.start_bit;
  }
}

// Hash function for strings to replace Qt's qHash
size_t qHash(const std::string &str) {
  size_t hash = 0;
  for (char c : str) {
    hash = hash * 31 + static_cast<unsigned char>(c);
  }
  return hash;
}
