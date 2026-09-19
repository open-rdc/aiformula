#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VISION_LANE_PLANNER_EXPORT __attribute__((dllexport))
#define VISION_LANE_PLANNER_IMPORT __attribute__((dllimport))
#else
#define VISION_LANE_PLANNER_EXPORT __declspec(dllexport)
#define VISION_LANE_PLANNER_IMPORT __declspec(dllimport)
#endif
#ifdef VISION_LANE_PLANNER_BUILDING_LIBRARY
#define VISION_LANE_PLANNER_PUBLIC VISION_LANE_PLANNER_EXPORT
#else
#define VISION_LANE_PLANNER_PUBLIC VISION_LANE_PLANNER_IMPORT
#endif
#define VISION_LANE_PLANNER_PUBLIC_TYPE VISION_LANE_PLANNER_PUBLIC
#define VISION_LANE_PLANNER_LOCAL
#else
#define VISION_LANE_PLANNER_EXPORT __attribute__((visibility("default")))
#define VISION_LANE_PLANNER_IMPORT
#if __GNUC__ >= 4
#define VISION_LANE_PLANNER_PUBLIC __attribute__((visibility("default")))
#define VISION_LANE_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else
#define VISION_LANE_PLANNER_PUBLIC
#define VISION_LANE_PLANNER_LOCAL
#endif
#define VISION_LANE_PLANNER_PUBLIC_TYPE
#endif
