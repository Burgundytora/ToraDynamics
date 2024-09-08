#ifndef TORA_ABSTRACT_LOGGER_H_
#define TORA_ABSTRACT_LOGGER_H_
#include <string>

class AbstractLogger {
 public:
  virtual void info(const std::string &msg) = 0;

  virtual void error(const std::string &msg) = 0;

  virtual void debug(const std::string &msg) = 0;

  virtual void warning(const std::string &msg) = 0;

  virtual ~AbstractLogger() = default;
};

#endif
