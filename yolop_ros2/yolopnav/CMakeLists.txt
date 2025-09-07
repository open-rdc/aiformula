cmake_minimum_required(VERSION 3.8)
project(yolopnav)

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
find_package(Eigen3 REQUIRED)

add_library(yolopnav_node
  src/lane_line_publisher_node.cpp
  src/lane_pixel_finder.cpp
  src/lane_pixel_to_point.cpp
)
target_compile_features(yolopnav_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(yolopnav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(yolopnav_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  utilities
)
target_link_libraries(yolopnav_node Eigen3::Eigen)

target_compile_definitions(yolopnav_node PRIVATE "YOLOPNAV_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS yolopnav_node
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
  yolopnav_node
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