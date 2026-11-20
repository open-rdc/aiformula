#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TOPOMAP_SERVER_EXPORT __attribute__ ((dllexport))
    #define TOPOMAP_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define TOPOMAP_SERVER_EXPORT __declspec(dllexport)
    #define TOPOMAP_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TOPOMAP_SERVER_BUILDING_LIBRARY
    #define TOPOMAP_SERVER_PUBLIC TOPOMAP_SERVER_EXPORT
  #else
    #define TOPOMAP_SERVER_PUBLIC TOPOMAP_SERVER_IMPORT
  #endif
  #define TOPOMAP_SERVER_PUBLIC_TYPE TOPOMAP_SERVER_PUBLIC
  #define TOPOMAP_SERVER_LOCAL
#else
  #define TOPOMAP_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define TOPOMAP_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define TOPOMAP_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define TOPOMAP_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TOPOMAP_SERVER_PUBLIC
    #define TOPOMAP_SERVER_LOCAL
  #endif
  #define TOPOMAP_SERVER_PUBLIC_TYPE
#endif
