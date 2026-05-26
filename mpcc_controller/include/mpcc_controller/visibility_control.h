#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #define MPCC_CONTROLLER_EXPORT __declspec(dllexport)
  #define MPCC_CONTROLLER_IMPORT __declspec(dllimport)
#else
  #define MPCC_CONTROLLER_EXPORT __attribute__((visibility("default")))
  #define MPCC_CONTROLLER_IMPORT
#endif

#ifdef MPCC_CONTROLLER_BUILDING_LIBRARY
  #define MPCC_CONTROLLER_PUBLIC MPCC_CONTROLLER_EXPORT
#else
  #define MPCC_CONTROLLER_PUBLIC MPCC_CONTROLLER_IMPORT
#endif
