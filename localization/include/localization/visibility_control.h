#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LOCALIZATION_EXPORT __attribute__ ((dllexport))
    #define LOCALIZATION_IMPORT __attribute__ ((dllimport))
  #else
    #define LOCALIZATION_EXPORT __declspec(dllexport)
    #define LOCALIZATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef LOCALIZATION_BUILDING_LIBRARY
    #define LOCALIZATION_PUBLIC LOCALIZATION_EXPORT
  #else
    #define LOCALIZATION_PUBLIC LOCALIZATION_IMPORT
  #endif
  #define LOCALIZATION_PUBLIC_TYPE LOCALIZATION_PUBLIC
  #define LOCALIZATION_LOCAL
#else
  #define LOCALIZATION_EXPORT __attribute__ ((visibility("default")))
  #define LOCALIZATION_IMPORT
  #if __GNUC__ >= 4
    #define LOCALIZATION_PUBLIC __attribute__ ((visibility("default")))
    #define LOCALIZATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LOCALIZATION_PUBLIC
    #define LOCALIZATION_LOCAL
  #endif
  #define LOCALIZATION_PUBLIC_TYPE
#endif
