#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EKF_LOCALIZER_EXPORT __attribute__ ((dllexport))
    #define EKF_LOCALIZER_IMPORT __attribute__ ((dllimport))
  #else
    #define EKF_LOCALIZER_EXPORT __declspec(dllexport)
    #define EKF_LOCALIZER_IMPORT __declspec(dllimport)
  #endif
  #ifdef EKF_LOCALIZER_BUILDING_LIBRARY
    #define EKF_LOCALIZER_PUBLIC EKF_LOCALIZER_EXPORT
  #else
    #define EKF_LOCALIZER_PUBLIC EKF_LOCALIZER_IMPORT
  #endif
  #define EKF_LOCALIZER_PUBLIC_TYPE EKF_LOCALIZER_PUBLIC
  #define EKF_LOCALIZER_LOCAL
#else
  #define EKF_LOCALIZER_EXPORT __attribute__ ((visibility("default")))
  #define EKF_LOCALIZER_IMPORT
  #if __GNUC__ >= 4
    #define EKF_LOCALIZER_PUBLIC __attribute__ ((visibility("default")))
    #define EKF_LOCALIZER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EKF_LOCALIZER_PUBLIC
    #define EKF_LOCALIZER_LOCAL
  #endif
  #define EKF_LOCALIZER_PUBLIC_TYPE
#endif
