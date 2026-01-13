cmake_minimum_required(VERSION 3.8)
project(visual_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- include ----
include_directories(
  include
)

# ---- vo_path_publisher_node ----
add_executable(vo_path_publisher_node
  src/vo_path_publisher_node.cpp
)

ament_target_dependencies(vo_path_publisher_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ---- vo_follower_node ----
add_executable(vo_follower_node
  src/vo_follower_node.cpp
)

target_include_directories(vo_follower_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../utilities/include
)

ament_target_dependencies(vo_follower_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(vo_follower_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---- install ----
install(TARGETS
  vo_path_publisher_node
  vo_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/vo_to_csv.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
