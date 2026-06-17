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

ament_auto_add_library(frenet_planner_plugin SHARED
  src/plugins/frenet_planner_plugin.cpp)
target_compile_features(frenet_planner_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(frenet_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(vectormap_frenet_plugin SHARED
  src/plugins/vectormap_frenet_plugin.cpp)
target_compile_features(vectormap_frenet_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(vectormap_frenet_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(local_planner plugins.xml)

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
