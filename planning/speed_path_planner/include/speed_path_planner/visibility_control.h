#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SPEED_PATH_PLANNER_EXPORT __attribute__((dllexport))
#define SPEED_PATH_PLANNER_IMPORT __attribute__((dllimport))
#else
#define SPEED_PATH_PLANNER_EXPORT __declspec(dllexport)
#define SPEED_PATH_PLANNER_IMPORT __declspec(dllimport)
#endif
#ifdef SPEED_PATH_PLANNER_BUILDING_LIBRARY
#define SPEED_PATH_PLANNER_PUBLIC SPEED_PATH_PLANNER_EXPORT
#else
#define SPEED_PATH_PLANNER_PUBLIC SPEED_PATH_PLANNER_IMPORT
#endif
#define SPEED_PATH_PLANNER_PUBLIC_TYPE SPEED_PATH_PLANNER_PUBLIC
#define SPEED_PATH_PLANNER_LOCAL
#else
#define SPEED_PATH_PLANNER_EXPORT __attribute__((visibility("default")))
#define SPEED_PATH_PLANNER_IMPORT
#if __GNUC__ >= 4
#define SPEED_PATH_PLANNER_PUBLIC __attribute__((visibility("default")))
#define SPEED_PATH_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else
#define SPEED_PATH_PLANNER_PUBLIC
#define SPEED_PATH_PLANNER_LOCAL
#endif
#define SPEED_PATH_PLANNER_PUBLIC_TYPE
#endif
