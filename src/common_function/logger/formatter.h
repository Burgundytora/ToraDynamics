#ifndef TORA_FORMAT_STRING_H_
#define TORA_FORMAT_STRING_H_

#include <sstream>
#include <string>

class FormatString {
 public:
  template <typename T>
  static std::string Convert(const T &format) {
    std::ostringstream os;
    os << format;
    return os.str();
  };

  template <typename... Args, typename T>
  static std::string Convert(const std::string &format, const T param, const Args &...args) {
    return Convert(Replace(format, param), args...);
  };

  template <typename T>
  static std::string Replace(const std::string &format, const T &param) {
    std::ostringstream os;
    os << param;

    if (format.length() == 0) {
      return os.str();
    }

    size_t off = format.find('{');
    size_t end = format.find('}');

    if (off == std::string::npos || end == std::string::npos) {
      return format + ' ' + os.str();
    }

    size_t count = end - off + 1;
    std::string out = format;
    return out.replace(off, count, os.str());
  }
};

inline std::string Replace(std::string& content, const std::string& old_text, const std::string& new_text){
  auto pos = content.find(old_text);
  return content.replace(pos, old_text.length(), new_text);;
}

#endif
