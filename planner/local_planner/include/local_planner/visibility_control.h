#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LOCAL_PLANNER_EXPORT __attribute__ ((dllexport))
    #define LOCAL_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define LOCAL_PLANNER_EXPORT __declspec(dllexport)
    #define LOCAL_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef LOCAL_PLANNER_BUILDING_LIBRARY
    #define LOCAL_PLANNER_PUBLIC LOCAL_PLANNER_EXPORT
  #else
    #define LOCAL_PLANNER_PUBLIC LOCAL_PLANNER_IMPORT
  #endif
  #define LOCAL_PLANNER_PUBLIC_TYPE LOCAL_PLANNER_PUBLIC
  #define LOCAL_PLANNER_LOCAL
#else
  #define LOCAL_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define LOCAL_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define LOCAL_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define LOCAL_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LOCAL_PLANNER_PUBLIC
    #define LOCAL_PLANNER_LOCAL
  #endif
  #define LOCAL_PLANNER_PUBLIC_TYPE
#endif
