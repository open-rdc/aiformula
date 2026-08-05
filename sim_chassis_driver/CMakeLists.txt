cmake_minimum_required(VERSION 3.8)
project(sim_chassis_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(sim_chassis_driver_node SHARED
  src/sim_chassis_driver.cpp)
target_compile_features(sim_chassis_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(sim_chassis_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  steered_drive_msg
  utilities
  chassis_driver
)

target_compile_definitions(sim_chassis_driver_node PRIVATE "SIM_CHASSIS_DRIVER_BUILDING_LIBRARY")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(USE_SCOPED_HEADER_INSTALL_DIR TRUE)
ament_auto_package()
