#ifndef CHASSIS_DRIVER__VISIBILITY_CONTROL_H_
#define CHASSIS_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CHASSIS_DRIVER_EXPORT __attribute__ ((dllexport))
    #define CHASSIS_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define CHASSIS_DRIVER_EXPORT __declspec(dllexport)
    #define CHASSIS_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CHASSIS_DRIVER_BUILDING_LIBRARY
    #define CHASSIS_DRIVER_PUBLIC CHASSIS_DRIVER_EXPORT
  #else
    #define CHASSIS_DRIVER_PUBLIC CHASSIS_DRIVER_IMPORT
  #endif
  #define CHASSIS_DRIVER_PUBLIC_TYPE CHASSIS_DRIVER_PUBLIC
  #define CHASSIS_DRIVER_LOCAL
#else
  #define CHASSIS_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define CHASSIS_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define CHASSIS_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define CHASSIS_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CHASSIS_DRIVER_PUBLIC
    #define CHASSIS_DRIVER_LOCAL
  #endif
  #define CHASSIS_DRIVER_PUBLIC_TYPE
#endif

#endif  // CHASSIS_DRIVER__VISIBILITY_CONTROL_H_
