cmake_minimum_required(VERSION 3.8)
project(obstacle_detector_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(obstacle_detector_msgs REQUIRED)

add_executable(obstacle_detector_node
  src/obstacle_detector_node.cpp
)

target_compile_features(obstacle_detector_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(obstacle_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(obstacle_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  obstacle_detector_msgs
)

install(TARGETS obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()