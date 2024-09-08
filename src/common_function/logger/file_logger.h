#ifndef TORA_FILE_LOGGER_H_
#define TORA_FILE_LOGGER_H_

#include <fstream>

#include "abstract_logger.h"

class FileLogger : public AbstractLogger {
 public:
  explicit FileLogger(const std::string &file_name = std::filesystem::current_path().filename().string()) {
    file_name_ = file_name;
  }

  void info(const std::string &msg) override {
    //    out_stream_ << "[Info]" << msg << std::endl;
  }

  void error(const std::string &msg) override {
    //    out_stream_ << "[Error]" << msg << std::endl;
    std::ofstream stream(file_name_ + ".$ME");
    stream << msg << std::endl;
    stream.close();
  }

  void debug(const std::string &msg) override {
    //    out_stream_ << "[Debug]" << msg << std::endl;
  }

  void warning(const std::string &msg) override {
    //    out_stream_ << "[Warning]" << msg << std::endl;
  }

  ~FileLogger() override {
    //    out_stream_.close();
  }

 private:
  //  std::basic_ofstream<char, std::char_traits<char>> out_stream_;
  std::string file_name_;
};

#endif  // GOLDFLEX_SHARED_LOGGER_FILE_LOGGER_H_
