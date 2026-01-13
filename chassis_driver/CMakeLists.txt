cmake_minimum_required(VERSION 3.8)
project(chassis_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(aiformula_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(chassis_driver_node
  src/chassis_driver_node.cpp
  src/velplanner.cpp)
target_compile_features(chassis_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_auto_add_executable(caster_sim_node
  src/caster_sim.cpp
  src/velplanner.cpp
)
# 実行可能ノード
ament_auto_add_executable(debug_printer
  src/debug_printer.cpp
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(chassis_driver_node PRIVATE "CHASSIS_DRIVER_BUILDING_LIBRARY")

add_executable(caster_sample_collector_sim src/caster_sample_collector_sim.cpp)
ament_target_dependencies(caster_sample_collector_sim
  rclcpp
  aiformula_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

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
install(TARGETS
  caster_sim_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS caster_sample_collector_sim
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)

