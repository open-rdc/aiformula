cmake_minimum_required(VERSION 3.8)
project(chassis_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(ament_index_cpp REQUIRED)

# PyTorch dependencies (required for utilities) - using manual setup to avoid nvtoolsext issues
set(TORCH_INCLUDE_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/../utilities/libtorch/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../utilities/libtorch/include/torch/csrc/api/include"
)
set(TORCH_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../utilities/libtorch/lib")

# Manually set torch libraries without problematic targets
set(TORCH_LIBRARIES 
    "${TORCH_LIB_DIR}/libtorch.so"
    "${TORCH_LIB_DIR}/libtorch_cpu.so"
    "${TORCH_LIB_DIR}/libc10.so"
)

find_package(utilities REQUIRED)

add_library(chassis_driver_node src/chassis_driver_node.cpp)
target_compile_features(chassis_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(chassis_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(chassis_driver_node PRIVATE
  ${TORCH_INCLUDE_DIRS})
ament_target_dependencies(chassis_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  socketcan_interface_msg
  ament_index_cpp
  utilities
)

target_link_libraries(chassis_driver_node
  ${TORCH_LIBRARIES}
)

# 実行可能ノード
add_executable(debug_printer
  src/debug_printer.cpp
)
target_include_directories(debug_printer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(debug_printer PRIVATE
  ${TORCH_INCLUDE_DIRS})
ament_target_dependencies(debug_printer
  rclcpp
  socketcan_interface_msg
  utilities
)

target_link_libraries(debug_printer
  ${TORCH_LIBRARIES}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(chassis_driver_node PRIVATE "CHASSIS_DRIVER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS chassis_driver_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS debug_printer
  DESTINATION lib/${PROJECT_NAME}
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
  chassis_driver_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  socketcan_interface_msg
  ament_index_cpp
  utilities
)

ament_package()
