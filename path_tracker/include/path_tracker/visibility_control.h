#ifndef PATH_TRACKER__VISIBILITY_CONTROL_H_
#define PATH_TRACKER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PATH_TRACKER_EXPORT __attribute__ ((dllexport))
    #define PATH_TRACKER_IMPORT __attribute__ ((dllimport))
  #else
    #define PATH_TRACKER_EXPORT __declspec(dllexport)
    #define PATH_TRACKER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PATH_TRACKER_BUILDING_LIBRARY
    #define PATH_TRACKER_PUBLIC PATH_TRACKER_EXPORT
  #else
    #define PATH_TRACKER_PUBLIC PATH_TRACKER_IMPORT
  #endif
  #define PATH_TRACKER_PUBLIC_TYPE PATH_TRACKER_PUBLIC
  #define PATH_TRACKER_LOCAL
#else
  #define PATH_TRACKER_EXPORT __attribute__ ((visibility("default")))
  #define PATH_TRACKER_IMPORT
  #if __GNUC__ >= 4
    #define PATH_TRACKER_PUBLIC __attribute__ ((visibility("default")))
    #define PATH_TRACKER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PATH_TRACKER_PUBLIC
    #define PATH_TRACKER_LOCAL
  #endif
  #define PATH_TRACKER_PUBLIC_TYPE
#endif

#endif  // PATH_TRACKER__VISIBILITY_CONTROL_H_
