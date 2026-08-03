#ifndef SIM_CHASSIS_DRIVER__VISIBILITY_CONTROL_H_
#define SIM_CHASSIS_DRIVER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIM_CHASSIS_DRIVER_EXPORT __attribute__ ((dllexport))
    #define SIM_CHASSIS_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define SIM_CHASSIS_DRIVER_EXPORT __declspec(dllexport)
    #define SIM_CHASSIS_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIM_CHASSIS_DRIVER_BUILDING_LIBRARY
    #define SIM_CHASSIS_DRIVER_PUBLIC SIM_CHASSIS_DRIVER_EXPORT
  #else
    #define SIM_CHASSIS_DRIVER_PUBLIC SIM_CHASSIS_DRIVER_IMPORT
  #endif
  #define SIM_CHASSIS_DRIVER_PUBLIC_TYPE SIM_CHASSIS_DRIVER_PUBLIC
  #define SIM_CHASSIS_DRIVER_LOCAL
#else
  #define SIM_CHASSIS_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define SIM_CHASSIS_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define SIM_CHASSIS_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define SIM_CHASSIS_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIM_CHASSIS_DRIVER_PUBLIC
    #define SIM_CHASSIS_DRIVER_LOCAL
  #endif
  #define SIM_CHASSIS_DRIVER_PUBLIC_TYPE
#endif

#endif  // SIM_CHASSIS_DRIVER__VISIBILITY_CONTROL_H_
