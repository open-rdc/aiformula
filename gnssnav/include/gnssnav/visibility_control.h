#ifndef GNSSNAV__VISIBILITY_CONTROL_H_
#define GNSSNAV__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GNSSNAV_EXPORT __attribute__ ((dllexport))
    #define GNSSNAV_IMPORT __attribute__ ((dllimport))
  #else
    #define GNSSNAV_EXPORT __declspec(dllexport)
    #define GNSSNAV_IMPORT __declspec(dllimport)
  #endif
  #ifdef GNSSNAV_BUILDING_LIBRARY
    #define GNSSNAV_PUBLIC GNSSNAV_EXPORT
  #else
    #define GNSSNAV_PUBLIC GNSSNAV_IMPORT
  #endif
  #define GNSSNAV_PUBLIC_TYPE GNSSNAV_PUBLIC
  #define GNSSNAV_LOCAL
#else
  #define GNSSNAV_EXPORT __attribute__ ((visibility("default")))
  #define GNSSNAV_IMPORT
  #if __GNUC__ >= 4
    #define GNSSNAV_PUBLIC __attribute__ ((visibility("default")))
    #define GNSSNAV_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GNSSNAV_PUBLIC
    #define GNSSNAV_LOCAL
  #endif
  #define GNSSNAV_PUBLIC_TYPE
#endif

#endif  // GNSSNAV__VISIBILITY_CONTROL_H_
