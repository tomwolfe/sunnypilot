#include "tools/cabana/utils/util.h"

#include <algorithm>
#include <csignal>
#include <limits>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "common/util.h"

// SegmentTree for Raylib

void SegmentTree::build(const std::vector<std::pair<double, double>> &arr) {
  size = arr.size();
  tree.resize(4 * size);  // size of the tree is 4 times the size of the array
  if (size > 0) {
    build_tree(arr, 1, 0, size - 1);
  }
}

void SegmentTree::build_tree(const std::vector<std::pair<double, double>> &arr, int n, int left, int right) {
  if (left == right) {
    tree[n] = arr[left];
  } else {
    const int mid = (left + right) >> 1;
    build_tree(arr, 2 * n, left, mid);
    build_tree(arr, 2 * n + 1, mid + 1, right);
    tree[n] = {std::min(tree[2 * n].first, tree[2 * n + 1].first), std::max(tree[2 * n].second, tree[2 * n + 1].second)};
  }
}

std::pair<double, double> SegmentTree::get_minmax(int n, int left, int right, int range_left, int range_right) const {
  if (range_left > right || range_right < left)
    return {std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  if (range_left <= left && range_right >= right)
    return tree[n];
  int mid = (left + right) >> 1;
  auto l = get_minmax(2 * n, left, mid, range_left, range_right);
  auto r = get_minmax(2 * n + 1, mid + 1, right, range_left, range_right);
  return {std::min(l.first, r.first), std::max(l.second, r.second)};
}

// UnixSignalHandler for Raylib
// Static members need to be defined outside the class
int UnixSignalHandler::sig_fd[2] = {};

UnixSignalHandler::UnixSignalHandler() {
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sig_fd)) {
    printf("Couldn't create TERM socketpair\n");
    return;
  }

  std::signal(SIGINT, UnixSignalHandler::signalHandler);
  std::signal(SIGTERM, UnixSignalHandler::signalHandler);
}

UnixSignalHandler::~UnixSignalHandler() {
  ::close(sig_fd[0]);
  ::close(sig_fd[1]);
}

void UnixSignalHandler::signalHandler(int s) {
  ::write(sig_fd[0], &s, sizeof(s));
}

void UnixSignalHandler::handleSigTerm() {
  int tmp;
  ::read(sig_fd[1], &tmp, sizeof(tmp));

  printf("\nexiting...\n");
  // For raylib, there's no qApp, so we just need to handle exit differently
  exit(0);
}

namespace utils {

bool isDarkTheme() {
  // For now, default to false for raylib theme
  return false;
}

void setTheme(int theme) {
  // For raylib, theme management is different
  // This is a simplified placeholder implementation
  static int prev_theme = -1;
  if (theme != prev_theme) {
    prev_theme = theme;
    // Theme change handling for raylib would go here
  }
}

std::string formatSeconds(double sec, bool include_milliseconds, bool absolute_time) {
  auto duration = std::chrono::duration<double>(sec);
  auto time_t = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
  std::time_t t = static_cast<std::time_t>(time_t);
  auto tm = *std::localtime(&t);

  std::stringstream ss;
  if (absolute_time) {
    ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
  } else {
    if (sec > 3600) { // More than an hour
      ss << std::put_time(&tm, "%H:%M:%S");
    } else { // Less than an hour
      ss << std::put_time(&tm, "%M:%S");
    }
  }

  if (include_milliseconds) {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;
    ss << "." << std::setfill('0') << std::setw(3) << ms;
  }

  return ss.str();
}

}  // namespace utils

int num_decimals(double num) {
  // Convert to string and count decimal places
  std::stringstream ss;
  ss << std::fixed << std::setprecision(10) << num;
  std::string s = ss.str();

  size_t dot_pos = s.find('.');
  if (dot_pos == std::string::npos) {
    return 0;
  }

  // Remove trailing zeros
  std::string fractional_part = s.substr(dot_pos + 1);
  fractional_part.erase(fractional_part.find_last_not_of('0') + 1, std::string::npos);

  return static_cast<int>(fractional_part.length());
}

std::string signalToolTip(const cabana::Signal *sig) {
  std::stringstream ss;
  ss << sig->name << "<br /><span font-size:small\">"
     << "Start Bit: " << sig->start_bit << " Size: " << sig->size << "<br />"
     << "MSB: " << sig->msb << " LSB: " << sig->lsb << "<br />"
     << "Little Endian: " << (sig->is_little_endian ? "Y" : "N")
     << " Signed: " << (sig->is_signed ? "Y" : "N") << "</span>";
  return ss.str();
}

void initApp(int argc, char *argv[], bool disable_hidpi) {
  // setup signal handlers to exit gracefully
  std::signal(SIGINT, [](int s) {
    printf("Received SIGINT, exiting...\n");
    exit(0);
  });
  std::signal(SIGTERM, [](int s) {
    printf("Received SIGTERM, exiting...\n");
    exit(0);
  });
}
