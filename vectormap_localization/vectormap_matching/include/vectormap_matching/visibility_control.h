#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VECTORMAP_MATCHING_EXPORT __attribute__ ((dllexport))
    #define VECTORMAP_MATCHING_IMPORT __attribute__ ((dllimport))
  #else
    #define VECTORMAP_MATCHING_EXPORT __declspec(dllexport)
    #define VECTORMAP_MATCHING_IMPORT __declspec(dllimport)
  #endif
  #ifdef VECTORMAP_MATCHING_BUILDING_LIBRARY
    #define VECTORMAP_MATCHING_PUBLIC VECTORMAP_MATCHING_EXPORT
  #else
    #define VECTORMAP_MATCHING_PUBLIC VECTORMAP_MATCHING_IMPORT
  #endif
  #define VECTORMAP_MATCHING_PUBLIC_TYPE VECTORMAP_MATCHING_PUBLIC
  #define VECTORMAP_MATCHING_LOCAL
#else
  #define VECTORMAP_MATCHING_EXPORT __attribute__ ((visibility("default")))
  #define VECTORMAP_MATCHING_IMPORT
  #if __GNUC__ >= 4
    #define VECTORMAP_MATCHING_PUBLIC __attribute__ ((visibility("default")))
    #define VECTORMAP_MATCHING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VECTORMAP_MATCHING_PUBLIC
    #define VECTORMAP_MATCHING_LOCAL
  #endif
  #define VECTORMAP_MATCHING_PUBLIC_TYPE
#endif
