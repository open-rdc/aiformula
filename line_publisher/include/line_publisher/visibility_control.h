#ifndef LINE_PUBLISHER__VISIBILITY_CONTROL_H_
#define LINE_PUBLISHER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LINE_PUBLISHER_EXPORT __attribute__ ((dllexport))
    #define LINE_PUBLISHER_IMPORT __attribute__ ((dllimport))
  #else
    #define LINE_PUBLISHER_EXPORT __declspec(dllexport)
    #define LINE_PUBLISHER_IMPORT __declspec(dllimport)
  #endif
  #ifdef LINE_PUBLISHER_BUILDING_LIBRARY
    #define LINE_PUBLISHER_PUBLIC LINE_PUBLISHER_EXPORT
  #else
    #define LINE_PUBLISHER_PUBLIC LINE_PUBLISHER_IMPORT
  #endif
  #define LINE_PUBLISHER_PUBLIC_TYPE LINE_PUBLISHER_PUBLIC
  #define LINE_PUBLISHER_LOCAL
#else
  #define LINE_PUBLISHER_EXPORT __attribute__ ((visibility("default")))
  #define LINE_PUBLISHER_IMPORT
  #if __GNUC__ >= 4
    #define LINE_PUBLISHER_PUBLIC __attribute__ ((visibility("default")))
    #define LINE_PUBLISHER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LINE_PUBLISHER_PUBLIC
    #define LINE_PUBLISHER_LOCAL
  #endif
  #define LINE_PUBLISHER_PUBLIC_TYPE
#endif

#endif  // LINE_PUBLISHER__VISIBILITY_CONTROL_H_
