#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VECTORMAP_PLANNER_EXPORT __attribute__ ((dllexport))
    #define VECTORMAP_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define VECTORMAP_PLANNER_EXPORT __declspec(dllexport)
    #define VECTORMAP_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef VECTORMAP_PLANNER_BUILDING_LIBRARY
    #define VECTORMAP_PLANNER_PUBLIC VECTORMAP_PLANNER_EXPORT
  #else
    #define VECTORMAP_PLANNER_PUBLIC VECTORMAP_PLANNER_IMPORT
  #endif
  #define VECTORMAP_PLANNER_PUBLIC_TYPE VECTORMAP_PLANNER_PUBLIC
  #define VECTORMAP_PLANNER_LOCAL
#else
  #define VECTORMAP_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define VECTORMAP_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define VECTORMAP_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define VECTORMAP_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VECTORMAP_PLANNER_PUBLIC
    #define VECTORMAP_PLANNER_LOCAL
  #endif
  #define VECTORMAP_PLANNER_PUBLIC_TYPE
#endif
