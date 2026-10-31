#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OBJECT_DETECTION_EXPORT __attribute__((dllexport))
#define OBJECT_DETECTION_IMPORT __attribute__((dllimport))
#else
#define OBJECT_DETECTION_EXPORT __declspec(dllexport)
#define OBJECT_DETECTION_IMPORT __declspec(dllimport)
#endif
#ifdef OBJECT_DETECTION_BUILDING_LIBRARY
#define OBJECT_DETECTION_PUBLIC OBJECT_DETECTION_EXPORT
#else
#define OBJECT_DETECTION_PUBLIC OBJECT_DETECTION_IMPORT
#endif
#define OBJECT_DETECTION_PUBLIC_TYPE OBJECT_DETECTION_PUBLIC
#define OBJECT_DETECTION_LOCAL
#else
#define OBJECT_DETECTION_EXPORT __attribute__((visibility("default")))
#define OBJECT_DETECTION_IMPORT
#if __GNUC__ >= 4
#define OBJECT_DETECTION_PUBLIC __attribute__((visibility("default")))
#define OBJECT_DETECTION_LOCAL __attribute__((visibility("hidden")))
#else
#define OBJECT_DETECTION_PUBLIC
#define OBJECT_DETECTION_LOCAL
#endif
#define OBJECT_DETECTION_PUBLIC_TYPE
#endif
