#ifndef UNTITLED_LIBTOOL_LOGGER_H_
#define UNTITLED_LIBTOOL_LOGGER_H_
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "abstract_logger.h"
#include "file_logger.h"
#include "formatter.h"
#include "terminal_logger.h"

class LogUtil {
 public:
  template <typename... Args>
  static void Warning(const std::string &prefix, const std::string &fmt, const Args &...args) {
    auto msg = ConvertString(fmt, args...);

    for (const auto &logger : logger_) {
      logger->warning(prefix + "" + msg);
    }
  }

  template <typename... Args>
  static void Debug(const std::string &prefix, const std::string &fmt, const Args &...args) {
    auto msg = ConvertString(fmt, args...);

    for (const auto &logger : logger_) {
      logger->debug(prefix + "" + msg);
    }
  }

  template <typename... Args>
  static void Info(const std::string &prefix, const std::string &fmt, const Args &...args) {
    auto msg = ConvertString(fmt, args...);

    for (const auto &logger : logger_) {
      logger->info(prefix + "" + msg);
    }
  }

  template <typename... Args>
  static void Error(const std::string &prefix, const std::string &fmt, const Args &...args) {
    auto msg = ConvertString(fmt, args...);

    for (const auto &logger : logger_) {
      logger->error(prefix + "" + msg);
    }
    throw std::logic_error(msg.c_str());
  }

  static void SetLogger(AbstractLogger *loggers) {
    //    LogUtil::logger_.clear();
    //    LogUtil::logger_.emplace_back(loggers);
  }

  template <typename... Args>
  static std::string ConvertString(const std::string &fmt, const Args &...args) {
    return FormatString::Convert("msg:{}", fmt, args...);
  }

  static std::string GetLoggingPrefix(const std::string &file, int line, bool with_time = true) {
    if (with_time) {
      time_t t = time(nullptr);
      char buf[32] = {0};
      tm *local = localtime(&t);  //转为本地时间
      strftime(buf, sizeof(buf) - 1, "%Y-%m-%d %H:%M:%S", local);
      auto file_name = std::filesystem::directory_entry(file).path().filename().string();
      return FormatString::Convert("[{}][{}][no.{}] ", buf, file_name, line);
    } else {
      auto file_name = std::filesystem::directory_entry(file).path().filename().string();
      return FormatString::Convert("[{}][no.{}] ", file_name, line);
    }
  }

 private:
  static inline std::vector<AbstractLogger *> logger_{new FileLogger(), new TerminalLogger()};
};

#endif
