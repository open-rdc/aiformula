cmake_minimum_required(VERSION 3.8)
project(speed_path_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/SpeedPathPoint.msg"
  "msg/SpeedPath.msg"
  "msg/StopPoint.msg"
)

set(_python_pkg_path "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/${PROJECT_NAME}")
if(EXISTS "${_python_pkg_path}" AND IS_DIRECTORY "${_python_pkg_path}")
  file(REMOVE_RECURSE "${_python_pkg_path}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
