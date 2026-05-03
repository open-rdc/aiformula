cmake_minimum_required(VERSION 3.8)
project(vectormap_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Server node library
ament_auto_add_library(vectormap_controller_server SHARED
  src/vectormap_controller_server.cpp
)
target_compile_features(vectormap_controller_server PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(vectormap_controller_server PRIVATE "VECTORMAP_CONTROL_BUILDING_LIBRARY")
target_include_directories(vectormap_controller_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Pure Pursuit plugin library
ament_auto_add_library(pure_pursuit_plugin SHARED
  src/plugins/pure_pursuit_plugin.cpp
)
target_compile_features(pure_pursuit_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(pure_pursuit_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(vectormap_control plugins.xml)

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
