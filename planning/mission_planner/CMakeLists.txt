cmake_minimum_required(VERSION 3.8)
project(mission_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(mission_planner_node SHARED
  src/mission_planner_node.cpp
  src/route_builder.cpp
)
target_compile_features(mission_planner_node PUBLIC cxx_std_17)
target_compile_definitions(mission_planner_node PRIVATE "MISSION_PLANNER_BUILDING_LIBRARY")

ament_auto_package()
