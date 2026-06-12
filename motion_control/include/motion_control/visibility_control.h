#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOTION_CONTROL_EXPORT __attribute__ ((dllexport))
    #define MOTION_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define MOTION_CONTROL_EXPORT __declspec(dllexport)
    #define MOTION_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOTION_CONTROL_BUILDING_LIBRARY
    #define MOTION_CONTROL_PUBLIC MOTION_CONTROL_EXPORT
  #else
    #define MOTION_CONTROL_PUBLIC MOTION_CONTROL_IMPORT
  #endif
  #define MOTION_CONTROL_PUBLIC_TYPE MOTION_CONTROL_PUBLIC
  #define MOTION_CONTROL_LOCAL
#else
  #define MOTION_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define MOTION_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define MOTION_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define MOTION_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOTION_CONTROL_PUBLIC
    #define MOTION_CONTROL_LOCAL
  #endif
  #define MOTION_CONTROL_PUBLIC_TYPE
#endif
