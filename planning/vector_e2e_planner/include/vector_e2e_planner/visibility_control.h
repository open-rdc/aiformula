#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VECTOR_E2E_PLANNER_EXPORT __attribute__((dllexport))
#define VECTOR_E2E_PLANNER_IMPORT __attribute__((dllimport))
#else
#define VECTOR_E2E_PLANNER_EXPORT __declspec(dllexport)
#define VECTOR_E2E_PLANNER_IMPORT __declspec(dllimport)
#endif
#ifdef VECTOR_E2E_PLANNER_BUILDING_LIBRARY
#define VECTOR_E2E_PLANNER_PUBLIC VECTOR_E2E_PLANNER_EXPORT
#else
#define VECTOR_E2E_PLANNER_PUBLIC VECTOR_E2E_PLANNER_IMPORT
#endif
#define VECTOR_E2E_PLANNER_PUBLIC_TYPE VECTOR_E2E_PLANNER_PUBLIC
#define VECTOR_E2E_PLANNER_LOCAL
#else
#define VECTOR_E2E_PLANNER_EXPORT __attribute__((visibility("default")))
#define VECTOR_E2E_PLANNER_IMPORT
#if __GNUC__ >= 4
#define VECTOR_E2E_PLANNER_PUBLIC __attribute__((visibility("default")))
#define VECTOR_E2E_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else
#define VECTOR_E2E_PLANNER_PUBLIC
#define VECTOR_E2E_PLANNER_LOCAL
#endif
#define VECTOR_E2E_PLANNER_PUBLIC_TYPE
#endif
