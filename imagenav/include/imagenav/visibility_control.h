#ifndef IMAGENAV__VISIBILITY_CONTROL_H_
#define IMAGENAV__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGENAV_EXPORT __attribute__ ((dllexport))
    #define IMAGENAV_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGENAV_EXPORT __declspec(dllexport)
    #define IMAGENAV_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGENAV_BUILDING_LIBRARY
    #define IMAGENAV_PUBLIC IMAGENAV_EXPORT
  #else
    #define IMAGENAV_PUBLIC IMAGENAV_IMPORT
  #endif
  #define IMAGENAV_PUBLIC_TYPE IMAGENAV_PUBLIC
  #define IMAGENAV_LOCAL
#else
  #define IMAGENAV_EXPORT __attribute__ ((visibility("default")))
  #define IMAGENAV_IMPORT
  #if __GNUC__ >= 4
    #define IMAGENAV_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGENAV_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGENAV_PUBLIC
    #define IMAGENAV_LOCAL
  #endif
  #define IMAGENAV_PUBLIC_TYPE
#endif

#endif  // IMAGENAV__VISIBILITY_CONTROL_H_
