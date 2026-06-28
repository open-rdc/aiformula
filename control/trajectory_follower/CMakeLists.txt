cmake_minimum_required(VERSION 3.8)
project(trajectory_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

ament_auto_add_library(controller_server SHARED
  src/controller_server.cpp
)
target_compile_features(controller_server PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(controller_server PRIVATE "TRAJECTORY_FOLLOWER_BUILDING_LIBRARY")
target_include_directories(controller_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(pure_pursuit_plugin SHARED
  src/plugins/pure_pursuit_plugin.cpp
)
target_compile_features(pure_pursuit_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(pure_pursuit_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(pid_mpc_core SHARED
  src/mpc/longitudinal_pid.cpp
  src/mpc/lateral_mpc.cpp
)
target_compile_features(pid_mpc_core PUBLIC c_std_99 cxx_std_17)
target_include_directories(pid_mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(pid_mpc_core Eigen3::Eigen)

ament_auto_add_library(pid_mpc_plugin SHARED
  src/plugins/pid_mpc_plugin.cpp
)
target_compile_features(pid_mpc_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(pid_mpc_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pid_mpc_plugin pid_mpc_core)

pluginlib_export_plugin_description_file(trajectory_follower plugins.xml)

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
