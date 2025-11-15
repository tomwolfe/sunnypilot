#include "tools/cabana/utils/export.h"

#include <fstream>
#include <iomanip>
#include <sstream>

#include "tools/cabana/streams/abstractstream.h"

namespace utils {

void exportToCSV(const std::string &file_name, std::optional<MessageId> msg_id) {
  std::ofstream file(file_name);
  if (file.is_open()) {
    file << "time,addr,bus,data\n";
    for (auto e : msg_id ? can->events(*msg_id) : can->allEvents()) {
      file << std::fixed << std::setprecision(3) << can->toSeconds(e->mono_time) << ","
           << "0x" << std::hex << e->address << std::dec << "," << static_cast<int>(e->src) << ","
           << "0x";

      // Convert data to hex string
      for (int i = 0; i < e->size; ++i) {
        file << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
             << static_cast<unsigned int>(e->dat[i]);
      }
      file << "\n";
    }
  }
}

void exportSignalsToCSV(const std::string &file_name, const MessageId &msg_id) {
  std::ofstream file(file_name);
  if (auto msg = dbc()->msg(msg_id); msg && msg->sigs.size() && file.is_open()) {
    file << "time,addr,bus";
    for (auto s : msg->sigs)
      file << "," << s->name;
    file << "\n";

    for (auto e : can->events(msg_id)) {
      file << std::fixed << std::setprecision(3) << can->toSeconds(e->mono_time) << ","
           << "0x" << std::hex << e->address << std::dec << "," << static_cast<int>(e->src);
      for (auto s : msg->sigs) {
        double value = 0;
        s->getValue(e->dat, e->size, &value);
        file << "," << std::fixed << std::setprecision(s->precision) << value;
      }
      file << "\n";
    }
  }
}

}  // namespace utils
