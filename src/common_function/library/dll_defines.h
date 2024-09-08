#ifndef TORA_DLL_DEFINES_H_
#define TORA_DLL_DEFINES_H_

// Generic helper definitions for shared library support
#if defined _WIN32
#define DLL_IMPORT __declspec(dllimport)
#define DLL_EXPORT __declspec(dllexport)
#define DLL_LOCAL
#define FUN_CDECL __cdecl
#else
#define DLL_IMPORT __attribute__((visibility("default")))
#define DLL_EXPORT __attribute__((visibility("default")))
#define DLL_LOCAL __attribute__((visibility("hidden")))
#define FUN_CDECL
#endif

#endif  // DLL_DEFINES_H_
