#include "tools/cabana/dbc/dbcfile.h"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <regex>
#include <cassert>

DBCFile::DBCFile(const std::string &dbc_file_name) {
  std::ifstream file(dbc_file_name, std::ios::binary);
  if (file.is_open()) {
    // Extract base name from path
    std::filesystem::path p(dbc_file_name);
    name_ = p.stem().string();
    filename = dbc_file_name;

    // Read all content
    std::stringstream buffer;
    buffer << file.rdbuf();
    parse(buffer.str());
  } else {
    throw std::runtime_error("Failed to open file.");
  }
}

DBCFile::DBCFile(const std::string &name, const std::string &content) : name_(name), filename("") {
  parse(content);
}

bool DBCFile::save() {
  assert(!filename.empty());
  return writeContents(filename);
}

bool DBCFile::saveAs(const std::string &new_filename) {
  filename = new_filename;
  return save();
}

bool DBCFile::writeContents(const std::string &fn) {
  std::ofstream file(fn, std::ios::binary);
  if (file.is_open()) {
    std::string content = generateDBC();
    file.write(content.c_str(), content.size());
    return !file.fail();
  }
  return false;
}

void DBCFile::updateMsg(const MessageId &id, const std::string &name, uint32_t size, const std::string &node, const std::string &comment) {
  auto &m = msgs[id.address];
  m.address = id.address;
  m.name = name;
  m.size = size;
  m.transmitter = node.empty() ? std::string(DEFAULT_NODE_NAME) : node;
  m.comment = comment;
}

cabana::Msg *DBCFile::msg(uint32_t address) {
  auto it = msgs.find(address);
  return it != msgs.end() ? &it->second : nullptr;
}

cabana::Msg *DBCFile::msg(const std::string &name) {
  auto it = std::find_if(msgs.begin(), msgs.end(), [&name](auto &m) { return m.second.name == name; });
  return it != msgs.end() ? &(it->second) : nullptr;
}

cabana::Signal *DBCFile::signal(uint32_t address, const std::string &name) {
  auto m = msg(address);
  return m ? (cabana::Signal *)m->sig(name) : nullptr;
}

void DBCFile::parse(const std::string &content) {
  msgs.clear();

  int line_num = 0;
  std::string line;
  cabana::Msg *current_msg = nullptr;
  int multiplexor_cnt = 0;
  bool seen_first = false;
  int pos = 0;
  int start_pos = 0;
  int line_end_pos = 0;

  // Parse content line by line
  while (start_pos < content.length()) {
    line_end_pos = content.find('\n', start_pos);
    if (line_end_pos == std::string::npos) {
      line_end_pos = content.length();
    }
    std::string raw_line = content.substr(start_pos, line_end_pos - start_pos);

    // Remove carriage return if present
    if (!raw_line.empty() && raw_line.back() == '\r') {
      raw_line.pop_back();
    }

    line = raw_line;
    // Trim line
    size_t first = line.find_first_not_of(" \t");
    size_t last = line.find_last_not_of(" \t");
    if (first != std::string::npos && last != std::string::npos) {
      line = line.substr(first, last - first + 1);
    } else {
      line = "";
    }

    bool seen = true;
    try {
      if (line.substr(0, 4) == "BO_ ") {
        multiplexor_cnt = 0;
        current_msg = parseBO(line);
      } else if (line.substr(0, 4) == "SG_ ") {
        parseSG(line, current_msg, multiplexor_cnt);
      } else if (line.substr(0, 5) == "VAL_ ") {
        parseVAL(line);
      } else if (line.substr(0, 7) == "CM_ BO_") {
        parseCM_BO(line, content, raw_line, pos);
      } else if (line.substr(0, 7) == "CM_ SG_ ") {
        parseCM_SG(line, content, raw_line, pos);
      } else {
        seen = false;
      }
    } catch (std::exception &e) {
      throw std::runtime_error("[" + filename + ":" + std::to_string(line_num) + "]" +
                               e.what() + ": " + line);
    }

    ++line_num;
    start_pos = line_end_pos + 1; // Move past the newline
    pos = start_pos;

    if (seen) {
      seen_first = true;
    } else if (!seen_first) {
      header += raw_line + "\n";
    }
  }

  for (auto &[_, m] : msgs) {
    m.update();
  }
}

cabana::Msg *DBCFile::parseBO(const std::string &line) {
  // Regular expression to capture address, name, size and transmitter
  // Format: BO_ <address> <name> : <size> <transmitter>
  std::regex bo_regexp(R"(^BO_\s+(\w+)\s+(\w+)\s*:\s*(\w+)\s+(\w+))");
  std::smatch match;

  if (!std::regex_search(line, match, bo_regexp))
    throw std::runtime_error("Invalid BO_ line format");

  uint32_t address = std::stoul(match[1].str(), nullptr, 16); // Parse as hex
  if (msgs.count(address) > 0)
    throw std::runtime_error("Duplicate message address: " + std::to_string(address));

  // Create a new message object
  cabana::Msg *msg = &msgs[address];
  msg->address = address;
  msg->name = match[2].str();
  msg->size = std::stoul(match[3].str());
  // Trim whitespace from transmitter
  std::string transmitter = match[4].str();
  size_t first = transmitter.find_first_not_of(" \t");
  size_t last = transmitter.find_last_not_of(" \t");
  if (first != std::string::npos && last != std::string::npos) {
    transmitter = transmitter.substr(first, last - first + 1);
  }
  msg->transmitter = transmitter;
  return msg;
}

void DBCFile::parseCM_BO(const std::string &line, const std::string &content, const std::string &raw_line, int &stream_pos) {
  std::regex msg_comment_regexp(R"(^CM_\s+BO_\s+(\w+)\s+\"((?:[^"\\]|\\.)*)\"\s*;)");
  std::smatch match;

  std::string parse_line = line; // Use the current line directly

  if (std::regex_search(parse_line, match, msg_comment_regexp)) {
    uint32_t address = std::stoul(match[1].str(), nullptr, 16);
    if (auto m = (cabana::Msg *)msg(address)) {
      std::string comment = match[2].str();
      // Trim whitespace
      size_t first = comment.find_first_not_of(" \t");
      size_t last = comment.find_last_not_of(" \t");
      if (first != std::string::npos && last != std::string::npos) {
        comment = comment.substr(first, last - first + 1);
      }
      // Replace escape sequences
      size_t pos = 0;
      while ((pos = comment.find("\\\"", pos)) != std::string::npos) {
        comment.replace(pos, 2, "\"");
        pos += 1;
      }
      m->comment = comment;
    } else {
      throw std::runtime_error("Invalid message comment format");
    }
  } else {
    throw std::runtime_error("Invalid message comment format");
  }
}

void DBCFile::parseSG(const std::string &line, cabana::Msg *current_msg, int &multiplexor_cnt) {
  std::regex sg_regexp(R"(^SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d+)([\+|\-])\s+\(([0-9.+\-eE]+),([0-9.+\-eE]+)\)\s+\[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\]\s+\"(.*)\"\s+(.*))");
  std::regex sgm_regexp(R"(^SG_\s+(\w+)\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d+)([\+|\-])\s+\(([0-9.+\-eE]+),([0-9.+\-eE]+)\)\s+\[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\]\s+\"(.*)\"\s+(.*))");

  if (!current_msg)
    throw std::runtime_error("No Message");

  int offset = 0;
  std::smatch match;
  if (std::regex_search(line, match, sg_regexp)) {
    // Match found with sg_regexp, offset remains 0
  } else if (std::regex_search(line, match, sgm_regexp)) {
    // Match found with sgm_regexp, offset becomes 1
    offset = 1;
  } else {
    throw std::runtime_error("Invalid SG_ line format");
  }

  std::string name = match[1].str();
  if (current_msg->sig(name) != nullptr)
    throw std::runtime_error("Duplicate signal name");

  cabana::Signal s{};
  if (offset == 1) {
    auto indicator = match[2].str();
    if (indicator == "M") {
      ++multiplexor_cnt;
      // Only one signal within a single message can be the multiplexer switch.
      if (multiplexor_cnt >= 2)
        throw std::runtime_error("Multiple multiplexor");

      s.type = cabana::Signal::Type::Multiplexor;
    } else {
      s.type = cabana::Signal::Type::Multiplexed;
      s.multiplex_value = std::stoi(indicator.substr(1));
    }
  }
  s.name = name;
  s.start_bit = std::stoi(match[offset + 2].str());
  s.size = std::stoi(match[offset + 3].str());
  s.is_little_endian = std::stoi(match[offset + 4].str()) == 1;
  s.is_signed = match[offset + 5].str() == "-";
  s.factor = std::stod(match[offset + 6].str());
  s.offset = std::stod(match[offset + 7].str());
  s.min = std::stod(match[8 + offset].str());
  s.max = std::stod(match[9 + offset].str());
  s.unit = match[10 + offset].str();
  // Trim whitespace from receiver_name
  std::string receiver_name = match[11 + offset].str();
  size_t first = receiver_name.find_first_not_of(" \t");
  size_t last = receiver_name.find_last_not_of(" \t");
  if (first != std::string::npos && last != std::string::npos) {
    receiver_name = receiver_name.substr(first, last - first + 1);
  } else {
    receiver_name = "";
  }
  s.receiver_name = receiver_name;
  current_msg->sigs.push_back(new cabana::Signal(s));
}

void DBCFile::parseCM_SG(const std::string &line, const std::string &content, const std::string &raw_line, int &stream_pos) {
  std::regex sg_comment_regexp(R"(^CM_\s+SG_\s+(\w+)\s+(\w+)\s+\"((?:[^"\\]|\\.)*)\"\s*;)");
  std::smatch match;

  std::string parse_line = line; // Use the current line directly

  if (std::regex_search(parse_line, match, sg_comment_regexp)) {
    uint32_t address = std::stoul(match[1].str(), nullptr, 16);
    if (auto s = signal(address, match[2].str())) {
      std::string comment = match[3].str();
      // Trim whitespace
      size_t first = comment.find_first_not_of(" \t");
      size_t last = comment.find_last_not_of(" \t");
      if (first != std::string::npos && last != std::string::npos) {
        comment = comment.substr(first, last - first + 1);
      }
      // Replace escape sequences
      size_t pos = 0;
      while ((pos = comment.find("\\\"", pos)) != std::string::npos) {
        comment.replace(pos, 2, "\"");
        pos += 1;
      }
      s->comment = comment;
    } else {
      throw std::runtime_error("Invalid CM_ SG_ line format");
    }
  } else {
    throw std::runtime_error("Invalid CM_ SG_ line format");
  }
}

void DBCFile::parseVAL(const std::string &line) {
  std::regex val_regexp(R"(VAL_\s+(\w+)\s+(\w+)\s+((?:\s*[-+]?\d+(?:\.\d+)?\s+\".+?\"[^;]*)+))");
  std::smatch match;

  if (std::regex_search(line, match, val_regexp)) {
    uint32_t address = std::stoul(match[1].str(), nullptr, 16);
    std::string name = match[2].str();

    if (auto s = signal(address, name)) {
      std::string desc_part = match[3].str();
      // Remove leading/trailing whitespace
      size_t first = desc_part.find_first_not_of(" \t");
      size_t last = desc_part.find_last_not_of(" \t");
      if (first != std::string::npos && last != std::string::npos) {
        desc_part = desc_part.substr(first, last - first + 1);
      } else {
        desc_part = "";
      }

      // Split by quote characters to separate values and descriptions
      std::vector<std::string> parts;
      size_t start = 0;
      for (size_t pos = 0; pos < desc_part.length(); pos++) {
        if (desc_part[pos] == '"') {
          if (start < pos) {
            parts.push_back(desc_part.substr(start, pos - start));
          }
          start = pos + 1;
        } else if (pos == desc_part.length() - 1) {
          parts.push_back(desc_part.substr(start));
        }
      }

      for (size_t i = 0; i < parts.size(); i += 2) {
        if (i + 1 < parts.size()) {
          std::string value_str = parts[i];
          // Trim whitespace from value
          size_t first_val = value_str.find_first_not_of(" \t");
          size_t last_val = value_str.find_last_not_of(" \t");
          if (first_val != std::string::npos && last_val != std::string::npos) {
            value_str = value_str.substr(first_val, last_val - first_val + 1);
          } else {
            value_str = "";
          }

          if (!value_str.empty()) {
            double value = std::stod(value_str);
            std::string desc = parts[i + 1];
            // Trim whitespace from description
            size_t first_desc = desc.find_first_not_of(" \t");
            size_t last_desc = desc.find_last_not_of(" \t");
            if (first_desc != std::string::npos && last_desc != std::string::npos) {
              desc = desc.substr(first_desc, last_desc - first_desc + 1);
            } else {
              desc = "";
            }
            s->val_desc.push_back({value, desc});
          }
        }
      }
    }
  } else {
    throw std::runtime_error("invalid VAL_ line format");
  }
}

std::string DBCFile::generateDBC() {
  std::string messages_and_signals_str, comment, val_desc; // Renamed local variable
  for (const auto &[address, m] : msgs) {
    std::string transmitter = m.transmitter.empty() ? std::string(DEFAULT_NODE_NAME) : m.transmitter;
    messages_and_signals_str += "BO_ " + std::to_string(address) + " " + m.name + ": " + std::to_string(m.size) + " " + transmitter + "\n";
    if (!m.comment.empty()) {
      std::string escaped_comment = m.comment;
      size_t pos = 0;
      while ((pos = escaped_comment.find("\"", pos)) != std::string::npos) {
        escaped_comment.replace(pos, 1, "\\\"");
        pos += 2;
      }
      comment += "CM_ BO_ " + std::to_string(address) + " \"" + escaped_comment + "\";\n";
    }
    for (auto sig : m.getSignals()) {
      std::string multiplexer_indicator;
      if (sig->type == cabana::Signal::Type::Multiplexor) {
        multiplexer_indicator = "M ";
      } else if (sig->type == cabana::Signal::Type::Multiplexed) {
        multiplexer_indicator = "m" + std::to_string(sig->multiplex_value) + " ";
      }
      messages_and_signals_str += " SG_ " + sig->name + " " + multiplexer_indicator + ": " +
                        std::to_string(sig->start_bit) + "|" + std::to_string(sig->size) + "@" +
                        std::to_string(sig->is_little_endian ? 1 : 0) + (sig->is_signed ? "-" : "+") + " (" +
                        doubleToString(sig->factor) + "," + doubleToString(sig->offset) + ") [" +
                        doubleToString(sig->min) + "|" + doubleToString(sig->max) + "] \"" + sig->unit + "\" " +
                        (sig->receiver_name.empty() ? std::string(DEFAULT_NODE_NAME) : sig->receiver_name) + "\n";
      if (!sig->comment.empty()) {
        std::string escaped_comment = sig->comment;
        size_t pos = 0;
        while ((pos = escaped_comment.find("\"", pos)) != std::string::npos) {
          escaped_comment.replace(pos, 1, "\\\"");
          pos += 2;
        }
        comment += "CM_ SG_ " + std::to_string(address) + " " + sig->name + " \"" + escaped_comment + "\";\n";
      }
      if (!sig->val_desc.empty()) {
        std::string text;
        for (auto &[val, desc] : sig->val_desc) {
          if (!text.empty()) {
            text += " ";
          }
          text += std::to_string(static_cast<long long>(val)) + " \"" + desc + "\"";
        }
        val_desc += "VAL_ " + std::to_string(address) + " " + sig->name + " " + text + ";\n";
      }
    }
    messages_and_signals_str += "\n";
  }
  return header + messages_and_signals_str + comment + val_desc; // Corrected return
}
