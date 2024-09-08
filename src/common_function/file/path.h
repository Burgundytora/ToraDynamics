#ifndef TORA_PATH_H_
#define TORA_PATH_H_
#include <cstdlib>
#include <filesystem>
#include <string>

#include "logger/logger.h"

#ifndef WIN32
#include <unistd.h>
#endif

using std::string;

class FilePathUtil {
 public:
  FilePathUtil() = delete;

  //判断当前路径是否为绝对路径
  static bool IsAbsolutePath(const string &path) { return std::filesystem::path(path).is_absolute(); }

  // 从文件或路径中提取出文件名，不包括扩展名
  static string GetFileName(const string &file_path) { return std::filesystem::path(file_path).filename().string(); }

  //使用当前工作目录，及相对路径 生成绝对路径
  static string TransferToAbsolutePath(const string &relative_path) {
    try {
      return std::filesystem::canonical(relative_path).string();
    } catch (const std::exception &ex) {
      LOG_ERROR("transfer to absolution path failed, relative_path:{}, error:{}", relative_path, ex.what());
    }
    return "";
  }

  static string TransferToAbsolutePath(const string &path, const string &relative_path) {
    try {
      auto current = std::filesystem::current_path();
      std::filesystem::current_path(path);
      auto abs_path = std::filesystem::canonical(relative_path).string();
      std::filesystem::current_path(current);
      return abs_path;
    } catch (const std::exception &ex) {
      LOG_ERROR("transfer to absolution path failed, relative_path:{}, error:{}", relative_path, ex.what());
    }
    return "";
  }

  static std::string GetExeFile() {
#ifdef WIN32
    char *str;
    _get_pgmptr(&str);
    return str;
#else
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    return std::string(result, (count > 0) ? count : 0);
#endif
  }

  static std::string GetExePath() { return std::filesystem::path(GetExeFile()).parent_path().string(); }

  static std::string GetConfigFile() {
    return std::filesystem::path(GetExePath()).append("submodule.gf.json").string();
  }

  static std::string GetTemplateModel() {
    return std::filesystem::path(GetExePath()).append("gtsim_standard_template.gf.json").string();
  }

  /**
   * 判断指定目录的文件是否存在
   * @param file_path 文件路径
   * @return bool 文件是否存在
   */
  static bool FileExist(const string &file_path) { return std::filesystem::exists(file_path); }

  static bool FileNotExist(const string &file_path) { return !FileExist(file_path); }
  /**
   * 判断指定目录的文件是否存在
   * @param file_path 文件路径
   * @return bool 文件是否存在
   */
  static void RenameDirectory(const string &old_path, string new_path = "") {
    if (new_path.empty()) {
      new_path = old_path + "_back";
    }
    errno = 0;
    if (FileExist(new_path)) {
      if (std::filesystem::remove_all(new_path.c_str()) == 0) {
        LOG_ERROR("remove file error, file:{}, error no :{}", new_path, strerror(errno));
      }
    }

    rename(old_path.c_str(), new_path.c_str());
  }

  static void BackupAndMakeDirectory(const string &path) {
    if (FilePathUtil::FileExist(path)) {
      FilePathUtil::RenameDirectory(path);
    }
    MakeDirectory(path);
  }

  static void MakeDirectory(const string &path) {
    if (FilePathUtil::FileExist(path)) {
      return;
    }
    try {
      std::filesystem::create_directories(path);
    } catch (std::exception &e) {
      LOG_ERROR("Create directory failed , error:{}", e.what());
    }
  }

  static bool RemoveFiles(const std::string &path, const std::string &patterns) {
    if (patterns.length() > 0) {
      try {
        for (auto &p : std::filesystem::directory_iterator(path)) {
          if (p.is_regular_file() && p.path().filename().string().find(patterns) != std::string::npos) {
            std::filesystem::remove(p.path());
          }
        }
      } catch (std::exception &e) {
        LOG_ERROR("remove file failed , error:{}", e.what());
      }
    }
    return true;
  }
};

#endif
