cmake_minimum_required(VERSION 3.8)
project(mapless_planning_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB msg_sources RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_sources}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
