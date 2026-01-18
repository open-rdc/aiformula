#ifndef UTILITIES__VISIBILITY_CONTROL_H_
#define UTILITIES__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UTILITIES_EXPORT __attribute__ ((dllexport))
    #define UTILITIES_IMPORT __attribute__ ((dllimport))
  #else
    #define UTILITIES_EXPORT __declspec(dllexport)
    #define UTILITIES_IMPORT __declspec(dllimport)
  #endif
  #ifdef UTILITIES_BUILDING_LIBRARY
    #define UTILITIES_PUBLIC UTILITIES_EXPORT
  #else
    #define UTILITIES_PUBLIC UTILITIES_IMPORT
  #endif
  #define UTILITIES_PUBLIC_TYPE UTILITIES_PUBLIC
  #define UTILITIES_LOCAL
#else
  #define UTILITIES_EXPORT __attribute__ ((visibility("default")))
  #define UTILITIES_IMPORT
  #if __GNUC__ >= 4
    #define UTILITIES_PUBLIC __attribute__ ((visibility("default")))
    #define UTILITIES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UTILITIES_PUBLIC
    #define UTILITIES_LOCAL
  #endif
  #define UTILITIES_PUBLIC_TYPE
#endif

#endif  // UTILITIES__VISIBILITY_CONTROL_H_
