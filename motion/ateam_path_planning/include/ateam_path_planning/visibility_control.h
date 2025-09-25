#ifndef ATEAM_PATH_PLANNING__VISIBILITY_CONTROL_H_
#define ATEAM_PATH_PLANNING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ATEAM_PATH_PLANNING_EXPORT __attribute__ ((dllexport))
    #define ATEAM_PATH_PLANNING_IMPORT __attribute__ ((dllimport))
  #else
    #define ATEAM_PATH_PLANNING_EXPORT __declspec(dllexport)
    #define ATEAM_PATH_PLANNING_IMPORT __declspec(dllimport)
  #endif
  #ifdef ATEAM_PATH_PLANNING_BUILDING_LIBRARY
    #define ATEAM_PATH_PLANNING_PUBLIC ATEAM_PATH_PLANNING_EXPORT
  #else
    #define ATEAM_PATH_PLANNING_PUBLIC ATEAM_PATH_PLANNING_IMPORT
  #endif
  #define ATEAM_PATH_PLANNING_PUBLIC_TYPE ATEAM_PATH_PLANNING_PUBLIC
  #define ATEAM_PATH_PLANNING_LOCAL
#else
  #define ATEAM_PATH_PLANNING_EXPORT __attribute__ ((visibility("default")))
  #define ATEAM_PATH_PLANNING_IMPORT
  #if __GNUC__ >= 4
    #define ATEAM_PATH_PLANNING_PUBLIC __attribute__ ((visibility("default")))
    #define ATEAM_PATH_PLANNING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ATEAM_PATH_PLANNING_PUBLIC
    #define ATEAM_PATH_PLANNING_LOCAL
  #endif
  #define ATEAM_PATH_PLANNING_PUBLIC_TYPE
#endif

#endif  // ATEAM_PATH_PLANNING__VISIBILITY_CONTROL_H_
