#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GLOBAL_PLANNER_EXPORT __attribute__ ((dllexport))
    #define GLOBAL_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define GLOBAL_PLANNER_EXPORT __declspec(dllexport)
    #define GLOBAL_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef GLOBAL_PLANNER_BUILDING_LIBRARY
    #define GLOBAL_PLANNER_PUBLIC GLOBAL_PLANNER_EXPORT
  #else
    #define GLOBAL_PLANNER_PUBLIC GLOBAL_PLANNER_IMPORT
  #endif
  #define GLOBAL_PLANNER_PUBLIC_TYPE GLOBAL_PLANNER_PUBLIC
  #define GLOBAL_PLANNER_LOCAL
#else
  #define GLOBAL_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define GLOBAL_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define GLOBAL_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define GLOBAL_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GLOBAL_PLANNER_PUBLIC
    #define GLOBAL_PLANNER_LOCAL
  #endif
  #define GLOBAL_PLANNER_PUBLIC_TYPE
#endif
