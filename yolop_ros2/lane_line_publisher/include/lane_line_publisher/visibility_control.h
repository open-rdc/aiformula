#ifndef LANE_LINE_PUBLISHER__VISIBILITY_CONTROL_H_
#define LANE_LINE_PUBLISHER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LANE_LINE_PUBLISHER_EXPORT __attribute__ ((dllexport))
    #define LANE_LINE_PUBLISHER_IMPORT __attribute__ ((dllimport))
  #else
    #define LANE_LINE_PUBLISHER_EXPORT __declspec(dllexport)
    #define LANE_LINE_PUBLISHER_IMPORT __declspec(dllimport)
  #endif
  #ifdef LANE_LINE_PUBLISHER_BUILDING_LIBRARY
    #define LANE_LINE_PUBLISHER_PUBLIC LANE_LINE_PUBLISHER_EXPORT
  #else
    #define LANE_LINE_PUBLISHER_PUBLIC LANE_LINE_PUBLISHER_IMPORT
  #endif
  #define LANE_LINE_PUBLISHER_PUBLIC_TYPE LANE_LINE_PUBLISHER_PUBLIC
  #define LANE_LINE_PUBLISHER_LOCAL
#else
  #define LANE_LINE_PUBLISHER_EXPORT __attribute__ ((visibility("default")))
  #define LANE_LINE_PUBLISHER_IMPORT
  #if __GNUC__ >= 4
    #define LANE_LINE_PUBLISHER_PUBLIC __attribute__ ((visibility("default")))
    #define LANE_LINE_PUBLISHER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LANE_LINE_PUBLISHER_PUBLIC
    #define LANE_LINE_PUBLISHER_LOCAL
  #endif
  #define LANE_LINE_PUBLISHER_PUBLIC_TYPE
#endif

#endif