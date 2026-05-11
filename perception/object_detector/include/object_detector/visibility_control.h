#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OBJECT_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define OBJECT_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define OBJECT_DETECTOR_EXPORT __declspec(dllexport)
    #define OBJECT_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef OBJECT_DETECTOR_BUILDING_LIBRARY
    #define OBJECT_DETECTOR_PUBLIC OBJECT_DETECTOR_EXPORT
  #else
    #define OBJECT_DETECTOR_PUBLIC OBJECT_DETECTOR_IMPORT
  #endif
  #define OBJECT_DETECTOR_PUBLIC_TYPE OBJECT_DETECTOR_PUBLIC
  #define OBJECT_DETECTOR_LOCAL
#else
  #define OBJECT_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define OBJECT_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define OBJECT_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define OBJECT_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OBJECT_DETECTOR_PUBLIC
    #define OBJECT_DETECTOR_LOCAL
  #endif
  #define OBJECT_DETECTOR_PUBLIC_TYPE
#endif
