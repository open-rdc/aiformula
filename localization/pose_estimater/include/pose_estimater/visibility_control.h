#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POSE_ESTIMATER_EXPORT __attribute__ ((dllexport))
    #define POSE_ESTIMATER_IMPORT __attribute__ ((dllimport))
  #else
    #define POSE_ESTIMATER_EXPORT __declspec(dllexport)
    #define POSE_ESTIMATER_IMPORT __declspec(dllimport)
  #endif
  #ifdef POSE_ESTIMATER_BUILDING_LIBRARY
    #define POSE_ESTIMATER_PUBLIC POSE_ESTIMATER_EXPORT
  #else
    #define POSE_ESTIMATER_PUBLIC POSE_ESTIMATER_IMPORT
  #endif
  #define POSE_ESTIMATER_PUBLIC_TYPE POSE_ESTIMATER_PUBLIC
  #define POSE_ESTIMATER_LOCAL
#else
  #define POSE_ESTIMATER_EXPORT __attribute__ ((visibility("default")))
  #define POSE_ESTIMATER_IMPORT
  #if __GNUC__ >= 4
    #define POSE_ESTIMATER_PUBLIC __attribute__ ((visibility("default")))
    #define POSE_ESTIMATER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POSE_ESTIMATER_PUBLIC
    #define POSE_ESTIMATER_LOCAL
  #endif
  #define POSE_ESTIMATER_PUBLIC_TYPE
#endif
