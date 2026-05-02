#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VECTORMAP_CONTROL_EXPORT __attribute__ ((dllexport))
    #define VECTORMAP_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define VECTORMAP_CONTROL_EXPORT __declspec(dllexport)
    #define VECTORMAP_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef VECTORMAP_CONTROL_BUILDING_LIBRARY
    #define VECTORMAP_CONTROL_PUBLIC VECTORMAP_CONTROL_EXPORT
  #else
    #define VECTORMAP_CONTROL_PUBLIC VECTORMAP_CONTROL_IMPORT
  #endif
  #define VECTORMAP_CONTROL_PUBLIC_TYPE VECTORMAP_CONTROL_PUBLIC
  #define VECTORMAP_CONTROL_LOCAL
#else
  #define VECTORMAP_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define VECTORMAP_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define VECTORMAP_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define VECTORMAP_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VECTORMAP_CONTROL_PUBLIC
    #define VECTORMAP_CONTROL_LOCAL
  #endif
  #define VECTORMAP_CONTROL_PUBLIC_TYPE
#endif
