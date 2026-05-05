cmake_minimum_required(VERSION 3.8)
project(lane_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(lane_planner_node SHARED
  src/lane_planner_server.cpp
  src/plugins/vectormap_lane_planner_plugin.cpp
  src/plugins/mapless_lane_planner_plugin.cpp
)
target_compile_features(lane_planner_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(lane_planner_node PRIVATE "LANE_PLANNER_BUILDING_LIBRARY")
target_include_directories(lane_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
