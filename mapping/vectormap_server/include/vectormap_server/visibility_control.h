#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VECTORMAP_SERVER_EXPORT __attribute__ ((dllexport))
    #define VECTORMAP_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define VECTORMAP_SERVER_EXPORT __declspec(dllexport)
    #define VECTORMAP_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef VECTORMAP_SERVER_BUILDING_LIBRARY
    #define VECTORMAP_SERVER_PUBLIC VECTORMAP_SERVER_EXPORT
  #else
    #define VECTORMAP_SERVER_PUBLIC VECTORMAP_SERVER_IMPORT
  #endif
  #define VECTORMAP_SERVER_PUBLIC_TYPE VECTORMAP_SERVER_PUBLIC
  #define VECTORMAP_SERVER_LOCAL
#else
  #define VECTORMAP_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define VECTORMAP_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define VECTORMAP_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define VECTORMAP_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VECTORMAP_SERVER_PUBLIC
    #define VECTORMAP_SERVER_LOCAL
  #endif
  #define VECTORMAP_SERVER_PUBLIC_TYPE
#endif
