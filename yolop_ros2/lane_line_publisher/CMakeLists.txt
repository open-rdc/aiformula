cmake_minimum_required(VERSION 3.8)
project(lane_line_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(utilities REQUIRED)

add_library(lane_line_publisher_node
  src/lane_line_publisher_node.cpp
  src/lane_pixel_finder.cpp
)
target_compile_features(lane_line_publisher_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(lane_line_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lane_line_publisher_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  utilities
)


target_compile_definitions(lane_line_publisher_node PRIVATE "LANE_LINE_PUBLISHER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lane_line_publisher_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  lane_line_publisher_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  utilities
)

ament_package()