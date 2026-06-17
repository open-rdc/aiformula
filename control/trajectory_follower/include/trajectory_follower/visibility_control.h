#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJECTORY_FOLLOWER_EXPORT __attribute__ ((dllexport))
    #define TRAJECTORY_FOLLOWER_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJECTORY_FOLLOWER_EXPORT __declspec(dllexport)
    #define TRAJECTORY_FOLLOWER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJECTORY_FOLLOWER_BUILDING_LIBRARY
    #define TRAJECTORY_FOLLOWER_PUBLIC TRAJECTORY_FOLLOWER_EXPORT
  #else
    #define TRAJECTORY_FOLLOWER_PUBLIC TRAJECTORY_FOLLOWER_IMPORT
  #endif
  #define TRAJECTORY_FOLLOWER_PUBLIC_TYPE TRAJECTORY_FOLLOWER_PUBLIC
  #define TRAJECTORY_FOLLOWER_LOCAL
#else
  #define TRAJECTORY_FOLLOWER_EXPORT __attribute__ ((visibility("default")))
  #define TRAJECTORY_FOLLOWER_IMPORT
  #if __GNUC__ >= 4
    #define TRAJECTORY_FOLLOWER_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJECTORY_FOLLOWER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJECTORY_FOLLOWER_PUBLIC
    #define TRAJECTORY_FOLLOWER_LOCAL
  #endif
  #define TRAJECTORY_FOLLOWER_PUBLIC_TYPE
#endif
