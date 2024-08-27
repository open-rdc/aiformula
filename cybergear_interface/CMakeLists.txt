cmake_minimum_required(VERSION 3.8)
project(cybergear_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(utilities REQUIRED)

add_library(cybergear_interface_node
  src/cybergear_interface_node.cpp
  src/driver.cpp
)
target_compile_features(cybergear_interface_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(cybergear_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cybergear_interface_node
  rclcpp
  std_msgs
  geometry_msgs
  socketcan_interface_msg
  utilities
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cybergear_interface_node PRIVATE "CYBERGEAR_INTERFACE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS cybergear_interface_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  cybergear_interface_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  socketcan_interface_msg
  utilities
)

ament_package()
