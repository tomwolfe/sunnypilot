#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <memory>

enum class LogLevel {
  DEBUG = 0,
  INFO = 1,
  WARNING = 2,
  ERROR = 3,
  CRITICAL = 4
};

class Logger {
public:
  static Logger& instance();
  
  void setLogLevel(LogLevel level);
  void setLogFile(const std::string& filename);
  void enableConsoleOutput(bool enabled);
  
  template<typename... Args>
  void log(LogLevel level, const std::string& tag, const Args&... args);
  
  template<typename... Args>
  void debug(const std::string& tag, const Args&... args);
  
  template<typename... Args>
  void info(const std::string& tag, const Args&... args);
  
  template<typename... Args>
  void warning(const std::string& tag, const Args&... args);
  
  template<typename... Args>
  void error(const std::string& tag, const Args&... args);
  
  template<typename... Args>
  void critical(const std::string& tag, const Args&... args);

private:
  Logger() = default;
  ~Logger() = default;
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;
  
  template<typename... Args>
  void writeLog(LogLevel level, const std::string& tag, const Args&... args);
  
  std::string levelToString(LogLevel level);
  
  LogLevel min_level_ = LogLevel::INFO;
  std::unique_ptr<std::ofstream> log_file_;
  bool console_output_ = true;
  std::mutex log_mutex_;
};

// Inline implementation
inline Logger& Logger::instance() {
  static Logger instance;
  return instance;
}

inline void Logger::setLogLevel(LogLevel level) {
  min_level_ = level;
}

inline void Logger::setLogFile(const std::string& filename) {
  log_file_ = std::make_unique<std::ofstream>(filename, std::ios::app);
}

inline void Logger::enableConsoleOutput(bool enabled) {
  console_output_ = enabled;
}

template<typename... Args>
void Logger::log(LogLevel level, const std::string& tag, const Args&... args) {
  if (level < min_level_) return;
  writeLog(level, tag, args...);
}

template<typename... Args>
void Logger::debug(const std::string& tag, const Args&... args) {
  log(LogLevel::DEBUG, tag, args...);
}

template<typename... Args>
void Logger::info(const std::string& tag, const Args&... args) {
  log(LogLevel::INFO, tag, args...);
}

template<typename... Args>
void Logger::warning(const std::string& tag, const Args&... args) {
  log(LogLevel::WARNING, tag, args...);
}

template<typename... Args>
void Logger::error(const std::string& tag, const Args&... args) {
  log(LogLevel::ERROR, tag, args...);
}

template<typename... Args>
void Logger::critical(const std::string& tag, const Args&... args) {
  log(LogLevel::CRITICAL, tag, args...);
}

template<typename... Args>
void Logger::writeLog(LogLevel level, const std::string& tag, const Args&... args) {
  std::lock_guard<std::mutex> lock(log_mutex_);
  
  // Create timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  
  std::ostringstream oss;
  oss << "[" << levelToString(level) << "] "
      << "[" << tag << "] "
      << "[" << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") 
      << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
  
  ((oss << args), ...); // C++17 fold expression to print all arguments
  oss << std::endl;
  
  std::string log_message = oss.str();
  
  if (console_output_) {
    std::cout << log_message << std::flush;
  }
  
  if (log_file_ && log_file_->is_open()) {
    *log_file_ << log_message;
    log_file_->flush();
  }
}

inline std::string Logger::levelToString(LogLevel level) {
  switch (level) {
    case LogLevel::DEBUG:   return "DEBUG";
    case LogLevel::INFO:    return "INFO";
    case LogLevel::WARNING: return "WARNING";
    case LogLevel::ERROR:   return "ERROR";
    case LogLevel::CRITICAL:return "CRITICAL";
    default:                return "UNKNOWN";
  }
}