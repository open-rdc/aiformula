cmake_minimum_required(VERSION 3.8)
project(steered_drive_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SteeredDrive.msg"
)

# remove stale python package directory so symlink creation succeeds on rebuilds
set(_python_pkg_path "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/${PROJECT_NAME}")
if(EXISTS "${_python_pkg_path}" AND IS_DIRECTORY "${_python_pkg_path}")
  file(REMOVE_RECURSE "${_python_pkg_path}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
