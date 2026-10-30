cmake_minimum_required(VERSION 3.8)
project(speed_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(speed_profile SHARED
  src/speed_profile.cpp)
target_compile_features(speed_profile PUBLIC cxx_std_17)
target_compile_definitions(speed_profile PRIVATE "SPEED_PATH_PLANNER_BUILDING_LIBRARY")

ament_auto_add_library(speed_path_planner_node SHARED
  src/speed_path_planner_node.cpp)
target_compile_features(speed_path_planner_node PUBLIC cxx_std_17)
target_compile_definitions(speed_path_planner_node PRIVATE "SPEED_PATH_PLANNER_BUILDING_LIBRARY")
target_link_libraries(speed_path_planner_node speed_profile)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_speed_limit.cpp)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_speed_limit test/test_speed_limit.cpp)
  target_link_libraries(test_speed_limit speed_profile)
endif()

ament_auto_package()
