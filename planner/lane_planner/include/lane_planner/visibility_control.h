#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LANE_PLANNER_EXPORT __attribute__ ((dllexport))
    #define LANE_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define LANE_PLANNER_EXPORT __declspec(dllexport)
    #define LANE_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef LANE_PLANNER_BUILDING_LIBRARY
    #define LANE_PLANNER_PUBLIC LANE_PLANNER_EXPORT
  #else
    #define LANE_PLANNER_PUBLIC LANE_PLANNER_IMPORT
  #endif
  #define LANE_PLANNER_PUBLIC_TYPE LANE_PLANNER_PUBLIC
  #define LANE_PLANNER_LOCAL
#else
  #define LANE_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define LANE_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define LANE_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define LANE_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LANE_PLANNER_PUBLIC
    #define LANE_PLANNER_LOCAL
  #endif
  #define LANE_PLANNER_PUBLIC_TYPE
#endif
