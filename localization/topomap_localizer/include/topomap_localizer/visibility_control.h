#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TOPOMAP_LOCALIZER_EXPORT __attribute__ ((dllexport))
    #define TOPOMAP_LOCALIZER_IMPORT __attribute__ ((dllimport))
  #else
    #define TOPOMAP_LOCALIZER_EXPORT __declspec(dllexport)
    #define TOPOMAP_LOCALIZER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TOPOMAP_LOCALIZER_BUILDING_LIBRARY
    #define TOPOMAP_LOCALIZER_PUBLIC TOPOMAP_LOCALIZER_EXPORT
  #else
    #define TOPOMAP_LOCALIZER_PUBLIC TOPOMAP_LOCALIZER_IMPORT
  #endif
  #define TOPOMAP_LOCALIZER_PUBLIC_TYPE TOPOMAP_LOCALIZER_PUBLIC
  #define TOPOMAP_LOCALIZER_LOCAL
#else
  #define TOPOMAP_LOCALIZER_EXPORT __attribute__ ((visibility("default")))
  #define TOPOMAP_LOCALIZER_IMPORT
  #if __GNUC__ >= 4
    #define TOPOMAP_LOCALIZER_PUBLIC __attribute__ ((visibility("default")))
    #define TOPOMAP_LOCALIZER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TOPOMAP_LOCALIZER_PUBLIC
    #define TOPOMAP_LOCALIZER_LOCAL
  #endif
  #define TOPOMAP_LOCALIZER_PUBLIC_TYPE
#endif
