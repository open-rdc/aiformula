#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PATH_SMOOTHER_EXPORT __attribute__((dllexport))
#define PATH_SMOOTHER_IMPORT __attribute__((dllimport))
#else
#define PATH_SMOOTHER_EXPORT __declspec(dllexport)
#define PATH_SMOOTHER_IMPORT __declspec(dllimport)
#endif
#ifdef PATH_SMOOTHER_BUILDING_LIBRARY
#define PATH_SMOOTHER_PUBLIC PATH_SMOOTHER_EXPORT
#else
#define PATH_SMOOTHER_PUBLIC PATH_SMOOTHER_IMPORT
#endif
#define PATH_SMOOTHER_PUBLIC_TYPE PATH_SMOOTHER_PUBLIC
#define PATH_SMOOTHER_LOCAL
#else
#define PATH_SMOOTHER_EXPORT __attribute__((visibility("default")))
#define PATH_SMOOTHER_IMPORT
#if __GNUC__ >= 4
#define PATH_SMOOTHER_PUBLIC __attribute__((visibility("default")))
#define PATH_SMOOTHER_LOCAL __attribute__((visibility("hidden")))
#else
#define PATH_SMOOTHER_PUBLIC
#define PATH_SMOOTHER_LOCAL
#endif
#define PATH_SMOOTHER_PUBLIC_TYPE
#endif
