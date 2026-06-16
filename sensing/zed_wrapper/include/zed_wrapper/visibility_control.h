#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ZED_WRAPPER_EXPORT __attribute__((dllexport))
    #define ZED_WRAPPER_IMPORT __attribute__((dllimport))
  #else
    #define ZED_WRAPPER_EXPORT __declspec(dllexport)
    #define ZED_WRAPPER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ZED_WRAPPER_BUILDING_LIBRARY
    #define ZED_WRAPPER_PUBLIC ZED_WRAPPER_EXPORT
  #else
    #define ZED_WRAPPER_PUBLIC ZED_WRAPPER_IMPORT
  #endif
  #define ZED_WRAPPER_LOCAL
#else
  #if __GNUC__ >= 4
    #define ZED_WRAPPER_PUBLIC __attribute__((visibility("default")))
    #define ZED_WRAPPER_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define ZED_WRAPPER_PUBLIC
    #define ZED_WRAPPER_LOCAL
  #endif
  #define ZED_WRAPPER_EXPORT ZED_WRAPPER_PUBLIC
  #define ZED_WRAPPER_IMPORT
#endif
