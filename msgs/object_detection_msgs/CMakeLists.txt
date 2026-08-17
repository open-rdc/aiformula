cmake_minimum_required(VERSION 3.8)
project(object_detection_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB msg_sources RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_sources} DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
