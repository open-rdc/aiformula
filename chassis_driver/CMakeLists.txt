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

# PyTorch dependencies
set(Torch_DIR "/usr/local/lib/python3.10/dist-packages/torch/share/cmake/Torch")
find_package(Torch REQUIRED)

add_library(chassis_driver_node src/chassis_driver_node.cpp)
target_compile_features(chassis_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(chassis_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(chassis_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  socketcan_interface_msg
  ament_index_cpp
)

target_link_libraries(chassis_driver_node
  ${TORCH_LIBRARIES}
)

# 実行可能ノード (一時的にコメントアウト)
# add_executable(debug_printer
#   src/debug_printer.cpp
# )
# target_include_directories(debug_printer PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(debug_printer
#   rclcpp
#   socketcan_interface_msg
# )

# target_link_libraries(debug_printer
#   ${TORCH_LIBRARIES}
# )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(chassis_driver_node PRIVATE "CHASSIS_DRIVER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY weights
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS chassis_driver_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# install(
#   TARGETS debug_printer
#   DESTINATION lib/${PROJECT_NAME}
# )

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
)

ament_package()
