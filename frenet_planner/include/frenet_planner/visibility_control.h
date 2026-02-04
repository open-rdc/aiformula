#ifndef FRENET_PLANNER__VISIBILITY_CONTROL_H_
#define FRENET_PLANNER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FRENET_PLANNER_EXPORT __attribute__ ((dllexport))
    #define FRENET_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define FRENET_PLANNER_EXPORT __declspec(dllexport)
    #define FRENET_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef FRENET_PLANNER_BUILDING_LIBRARY
    #define FRENET_PLANNER_PUBLIC FRENET_PLANNER_EXPORT
  #else
    #define FRENET_PLANNER_PUBLIC FRENET_PLANNER_IMPORT
  #endif
  #define FRENET_PLANNER_PUBLIC_TYPE FRENET_PLANNER_PUBLIC
  #define FRENET_PLANNER_LOCAL
#else
  #define FRENET_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define FRENET_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define FRENET_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define FRENET_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FRENET_PLANNER_PUBLIC
    #define FRENET_PLANNER_LOCAL
  #endif
  #define FRENET_PLANNER_PUBLIC_TYPE
#endif

#endif  // FRENET_PLANNER__VISIBILITY_CONTROL_H_
