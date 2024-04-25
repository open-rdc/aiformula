#ifndef ROBOTEQ_DRIVER__VISIBILITY_CONTROL_H_
#define ROBOTEQ_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTEQ_DRIVER_EXPORT __attribute__ ((dllexport))
    #define ROBOTEQ_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTEQ_DRIVER_EXPORT __declspec(dllexport)
    #define ROBOTEQ_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTEQ_DRIVER_BUILDING_LIBRARY
    #define ROBOTEQ_DRIVER_PUBLIC ROBOTEQ_DRIVER_EXPORT
  #else
    #define ROBOTEQ_DRIVER_PUBLIC ROBOTEQ_DRIVER_IMPORT
  #endif
  #define ROBOTEQ_DRIVER_PUBLIC_TYPE ROBOTEQ_DRIVER_PUBLIC
  #define ROBOTEQ_DRIVER_LOCAL
#else
  #define ROBOTEQ_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTEQ_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTEQ_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTEQ_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTEQ_DRIVER_PUBLIC
    #define ROBOTEQ_DRIVER_LOCAL
  #endif
  #define ROBOTEQ_DRIVER_PUBLIC_TYPE
#endif

#endif  // ROBOTEQ_DRIVER__VISIBILITY_CONTROL_H_
