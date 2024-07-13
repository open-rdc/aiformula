#ifndef CYBERGEAR_INTERFACE__VISIBILITY_CONTROL_H_
#define CYBERGEAR_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CYBERGEAR_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define CYBERGEAR_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define CYBERGEAR_INTERFACE_EXPORT __declspec(dllexport)
    #define CYBERGEAR_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CYBERGEAR_INTERFACE_BUILDING_LIBRARY
    #define CYBERGEAR_INTERFACE_PUBLIC CYBERGEAR_INTERFACE_EXPORT
  #else
    #define CYBERGEAR_INTERFACE_PUBLIC CYBERGEAR_INTERFACE_IMPORT
  #endif
  #define CYBERGEAR_INTERFACE_PUBLIC_TYPE CYBERGEAR_INTERFACE_PUBLIC
  #define CYBERGEAR_INTERFACE_LOCAL
#else
  #define CYBERGEAR_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define CYBERGEAR_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define CYBERGEAR_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define CYBERGEAR_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CYBERGEAR_INTERFACE_PUBLIC
    #define CYBERGEAR_INTERFACE_LOCAL
  #endif
  #define CYBERGEAR_INTERFACE_PUBLIC_TYPE
#endif

#endif  // CYBERGEAR_INTERFACE__VISIBILITY_CONTROL_H_
