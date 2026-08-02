cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(local_planner_server SHARED
  src/local_planner_server.cpp)
target_compile_features(local_planner_server PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(local_planner_server PRIVATE "LOCAL_PLANNER_BUILDING_LIBRARY")
target_include_directories(local_planner_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(frenet_core SHARED
  src/frenet/polynomial.cpp
  src/frenet/frenet_planner.cpp
  src/frenet/hard_constraint.cpp
  src/frenet/soft_constraint.cpp)
target_compile_features(frenet_core PUBLIC c_std_99 cxx_std_17)
target_include_directories(frenet_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(frenet_planner_plugin SHARED
  src/plugins/frenet_planner_plugin.cpp)
target_compile_features(frenet_planner_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(frenet_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(frenet_planner_plugin frenet_core)

pluginlib_export_plugin_description_file(local_planner plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frenet_core test/test_frenet_core.cpp)
  target_link_libraries(test_frenet_core frenet_core)
  ament_auto_add_gtest(test_frenet_planner_plugin test/test_frenet_planner_plugin.cpp)
  target_link_libraries(test_frenet_planner_plugin frenet_planner_plugin)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package()
