#ifndef GTEST_DEMO_SRC_LOAD_DLL_H_
#define GTEST_DEMO_SRC_LOAD_DLL_H_
#include <string>

#include "file_path.h"
using std::string;

#if defined _WIN32
#include <Windows.h>
#else
#include "dlfcn.h"
#endif

class DllLoader {
 public:
  DllLoader(string dll_file) {
#if defined _WIN32
    handle_dll_ = LoadLibrary(dll_file.c_str());
#else
    handle_dll_ = dlopen(dll_file.c_str(), RTLD_LAZY);
#endif
    if (handle_dll_ == nullptr) {
#ifdef _WIN32
      LOG_ERROR("load library failed, dll file:{}, error: {}", dll_file, GetLastError());
#else
      LogUtil::Error("DllLoader", "load library failed, dll file:{}, error: {}", dll_file, dlerror());
#endif
    }
  }

  ~DllLoader() {
    if (handle_dll_ != nullptr) {
#if defined _WIN32
      ::FreeLibrary(handle_dll_);
#else
      ::dlclose(handle_dll_);
#endif
    }
  }

  template <typename T>
  T GetFunction(const string &name) {
#if defined _WIN32
    return (T)GetProcAddress(handle_dll_, name.c_str());
#else
    return (T)::dlsym(handle_dll_, name.c_str());
#endif
  }

 private:
#if defined _WIN32
  HINSTANCE handle_dll_{};
#else
  void *handle_dll_{};
#endif
};
#endif
