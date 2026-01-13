#ifndef YOLOPNAV__VISIBILITY_CONTROL_H_
#define YOLOPNAV__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define YOLOPNAV_EXPORT __attribute__ ((dllexport))
    #define YOLOPNAV_IMPORT __attribute__ ((dllimport))
  #else
    #define YOLOPNAV_EXPORT __declspec(dllexport)
    #define YOLOPNAV_IMPORT __declspec(dllimport)
  #endif
  #ifdef YOLOPNAV_BUILDING_LIBRARY
    #define YOLOPNAV_PUBLIC YOLOPNAV_EXPORT
  #else
    #define YOLOPNAV_PUBLIC YOLOPNAV_IMPORT
  #endif
  #define YOLOPNAV_PUBLIC_TYPE YOLOPNAV_PUBLIC
  #define YOLOPNAV_LOCAL
#else
  #define YOLOPNAV_EXPORT __attribute__ ((visibility("default")))
  #define YOLOPNAV_IMPORT
  #if __GNUC__ >= 4
    #define YOLOPNAV_PUBLIC __attribute__ ((visibility("default")))
    #define YOLOPNAV_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define YOLOPNAV_PUBLIC
    #define YOLOPNAV_LOCAL
  #endif
  #define YOLOPNAV_PUBLIC_TYPE
#endif

#endif