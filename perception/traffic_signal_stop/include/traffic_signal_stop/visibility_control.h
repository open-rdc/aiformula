#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TRAFFIC_SIGNAL_STOP_EXPORT __attribute__((dllexport))
#define TRAFFIC_SIGNAL_STOP_IMPORT __attribute__((dllimport))
#else
#define TRAFFIC_SIGNAL_STOP_EXPORT __declspec(dllexport)
#define TRAFFIC_SIGNAL_STOP_IMPORT __declspec(dllimport)
#endif
#ifdef TRAFFIC_SIGNAL_STOP_BUILDING_LIBRARY
#define TRAFFIC_SIGNAL_STOP_PUBLIC TRAFFIC_SIGNAL_STOP_EXPORT
#else
#define TRAFFIC_SIGNAL_STOP_PUBLIC TRAFFIC_SIGNAL_STOP_IMPORT
#endif
#define TRAFFIC_SIGNAL_STOP_PUBLIC_TYPE TRAFFIC_SIGNAL_STOP_PUBLIC
#define TRAFFIC_SIGNAL_STOP_LOCAL
#else
#define TRAFFIC_SIGNAL_STOP_EXPORT __attribute__((visibility("default")))
#define TRAFFIC_SIGNAL_STOP_IMPORT
#if __GNUC__ >= 4
#define TRAFFIC_SIGNAL_STOP_PUBLIC __attribute__((visibility("default")))
#define TRAFFIC_SIGNAL_STOP_LOCAL __attribute__((visibility("hidden")))
#else
#define TRAFFIC_SIGNAL_STOP_PUBLIC
#define TRAFFIC_SIGNAL_STOP_LOCAL
#endif
#define TRAFFIC_SIGNAL_STOP_PUBLIC_TYPE
#endif
