#ifndef TORA_LOGGER_H_
#define TORA_LOGGER_H_

#include "logger_proxy.h"
#define LOG_ERROR(fmt, ...) LogUtil::Error(LogUtil::GetLoggingPrefix(__FILE__, __LINE__), fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) LogUtil::Info(LogUtil::GetLoggingPrefix(__FILE__, __LINE__), fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) LogUtil::Debug(LogUtil::GetLoggingPrefix(__FILE__, __LINE__), fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) LogUtil::Warning(LogUtil::GetLoggingPrefix(__FILE__, __LINE__), fmt, ##__VA_ARGS__)
#define PRINTF(info) LOG_INFO(#info, ":{}", info)

#define ASSERT(condition, ...)                                \
  {                                                               \
    if (!(condition)) {                                           \
      LOG_ERROR("Assertion failed: {}", #condition, ##__VA_ARGS__); \
      exit(EXIT_FAILURE);                                         \
    }                                                             \
  }

#endif  // LIBRARY_LOGGER_LOGGER_H_
