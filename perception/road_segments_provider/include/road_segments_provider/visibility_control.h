#ifndef ROAD_SEGMENTS_PROVIDER__VISIBILITY_CONTROL_H_
#define ROAD_SEGMENTS_PROVIDER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROAD_SEGMENTS_PROVIDER_EXPORT __attribute__ ((dllexport))
    #define ROAD_SEGMENTS_PROVIDER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROAD_SEGMENTS_PROVIDER_EXPORT __declspec(dllexport)
    #define ROAD_SEGMENTS_PROVIDER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROAD_SEGMENTS_PROVIDER_BUILDING_LIBRARY
    #define ROAD_SEGMENTS_PROVIDER_PUBLIC ROAD_SEGMENTS_PROVIDER_EXPORT
  #else
    #define ROAD_SEGMENTS_PROVIDER_PUBLIC ROAD_SEGMENTS_PROVIDER_IMPORT
  #endif
  #define ROAD_SEGMENTS_PROVIDER_PUBLIC_TYPE ROAD_SEGMENTS_PROVIDER_PUBLIC
  #define ROAD_SEGMENTS_PROVIDER_LOCAL
#else
  #define ROAD_SEGMENTS_PROVIDER_EXPORT __attribute__ ((visibility("default")))
  #define ROAD_SEGMENTS_PROVIDER_IMPORT
  #if __GNUC__ >= 4
    #define ROAD_SEGMENTS_PROVIDER_PUBLIC __attribute__ ((visibility("default")))
    #define ROAD_SEGMENTS_PROVIDER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROAD_SEGMENTS_PROVIDER_PUBLIC
    #define ROAD_SEGMENTS_PROVIDER_LOCAL
  #endif
  #define ROAD_SEGMENTS_PROVIDER_PUBLIC_TYPE
#endif

#endif  // ROAD_SEGMENTS_PROVIDER__VISIBILITY_CONTROL_H_
