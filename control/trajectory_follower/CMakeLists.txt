cmake_minimum_required(VERSION 3.8)
project(trajectory_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

ament_auto_add_library(pid_mpc_core SHARED
  src/mpc/longitudinal_pid.cpp
  src/mpc/lateral_mpc.cpp
  src/mpc/speed_limit.cpp
)
target_compile_features(pid_mpc_core PUBLIC cxx_std_17)
target_link_libraries(pid_mpc_core Eigen3::Eigen)

ament_auto_add_library(controller_server SHARED
  src/controller_server.cpp
)
target_compile_features(controller_server PUBLIC cxx_std_17)
target_compile_definitions(controller_server PRIVATE "TRAJECTORY_FOLLOWER_BUILDING_LIBRARY")

ament_auto_add_library(pure_pursuit_plugin SHARED
  src/plugins/pure_pursuit_plugin.cpp
)
target_compile_features(pure_pursuit_plugin PUBLIC cxx_std_17)
target_link_libraries(pure_pursuit_plugin pid_mpc_core)

ament_auto_add_library(pid_mpc_plugin SHARED
  src/plugins/pid_mpc_plugin.cpp
)
target_compile_features(pid_mpc_plugin PUBLIC cxx_std_17)
target_link_libraries(pid_mpc_plugin pid_mpc_core)

pluginlib_export_plugin_description_file(trajectory_follower plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_speed_limit test/test_speed_limit.cpp)
  target_link_libraries(test_speed_limit pid_mpc_core)
  ament_add_gtest(test_longitudinal_pid test/test_longitudinal_pid.cpp)
  target_link_libraries(test_longitudinal_pid pid_mpc_core)
  ament_add_gtest(test_lateral_mpc test/test_lateral_mpc.cpp)
  target_link_libraries(test_lateral_mpc pid_mpc_core)
endif()

ament_auto_package()
