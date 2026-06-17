#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MISSION_PLANNER_EXPORT __attribute__ ((dllexport))
    #define MISSION_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define MISSION_PLANNER_EXPORT __declspec(dllexport)
    #define MISSION_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MISSION_PLANNER_BUILDING_LIBRARY
    #define MISSION_PLANNER_PUBLIC MISSION_PLANNER_EXPORT
  #else
    #define MISSION_PLANNER_PUBLIC MISSION_PLANNER_IMPORT
  #endif
  #define MISSION_PLANNER_PUBLIC_TYPE MISSION_PLANNER_PUBLIC
  #define MISSION_PLANNER_LOCAL
#else
  #define MISSION_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define MISSION_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define MISSION_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define MISSION_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MISSION_PLANNER_PUBLIC
    #define MISSION_PLANNER_LOCAL
  #endif
  #define MISSION_PLANNER_PUBLIC_TYPE
#endif
