cmake_minimum_required(VERSION 3.8)
project(visual_odometry)


# 警告設定（お好みで）
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

include_directories(
  include
)

# ---- ノードのビルド ----
add_executable(vo_path_publisher_node
  src/vo_path_publisher_node.cpp
)

ament_target_dependencies(vo_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(vo_follower_node
  src/vo_follower_node.cpp
)

ament_target_dependencies(vo_fo;;ower_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
install(TARGETS
  vo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
