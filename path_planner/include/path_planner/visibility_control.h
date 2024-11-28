#ifndef PATH_PLANNER__VISIBILITY_CONTROL_H_
#define PATH_PLANNER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PATH_PLANNER_EXPORT __attribute__ ((dllexport))
    #define PATH_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define PATH_PLANNER_EXPORT __declspec(dllexport)
    #define PATH_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PATH_PLANNER_BUILDING_LIBRARY
    #define PATH_PLANNER_PUBLIC PATH_PLANNER_EXPORT
  #else
    #define PATH_PLANNER_PUBLIC PATH_PLANNER_IMPORT
  #endif
  #define PATH_PLANNER_PUBLIC_TYPE PATH_PLANNER_PUBLIC
  #define PATH_PLANNER_LOCAL
#else
  #define PATH_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define PATH_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define PATH_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define PATH_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PATH_PLANNER_PUBLIC
    #define PATH_PLANNER_LOCAL
  #endif
  #define PATH_PLANNER_PUBLIC_TYPE
#endif

#endif  // PATH_PLANNER__VISIBILITY_CONTROL_H_
