#ifndef TORA_TERMINAL_LOGGER_H_
#define TORA_TERMINAL_LOGGER_H_
#include "abstract_logger.h"

class TerminalLogger : public AbstractLogger {
 public:
  void info(const std::string &msg) override { std::cout << "[Info]" << msg << std::endl; }

  void error(const std::string &msg) override { std::cerr << "[Error]" << msg << std::endl; }

  void debug(const std::string &msg) override { std::cout << "[Debug]" << msg << std::endl; }

  void warning(const std::string &msg) override { std::cout << "[Warning]" << msg << std::endl; }
};

#endif
