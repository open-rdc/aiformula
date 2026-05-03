#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VECTORMAP_LOCALIZATION_EXPORT __attribute__ ((dllexport))
    #define VECTORMAP_LOCALIZATION_IMPORT __attribute__ ((dllimport))
  #else
    #define VECTORMAP_LOCALIZATION_EXPORT __declspec(dllexport)
    #define VECTORMAP_LOCALIZATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef VECTORMAP_LOCALIZATION_BUILDING_LIBRARY
    #define VECTORMAP_LOCALIZATION_PUBLIC VECTORMAP_LOCALIZATION_EXPORT
  #else
    #define VECTORMAP_LOCALIZATION_PUBLIC VECTORMAP_LOCALIZATION_IMPORT
  #endif
  #define VECTORMAP_LOCALIZATION_PUBLIC_TYPE VECTORMAP_LOCALIZATION_PUBLIC
  #define VECTORMAP_LOCALIZATION_LOCAL
#else
  #define VECTORMAP_LOCALIZATION_EXPORT __attribute__ ((visibility("default")))
  #define VECTORMAP_LOCALIZATION_IMPORT
  #if __GNUC__ >= 4
    #define VECTORMAP_LOCALIZATION_PUBLIC __attribute__ ((visibility("default")))
    #define VECTORMAP_LOCALIZATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VECTORMAP_LOCALIZATION_PUBLIC
    #define VECTORMAP_LOCALIZATION_LOCAL
  #endif
  #define VECTORMAP_LOCALIZATION_PUBLIC_TYPE
#endif
