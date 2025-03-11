cmake_minimum_required(VERSION 3.8)
project(lane_line_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# auto find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(cv_bridge REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(lane_line_publisher
  src/bird_eye_view_node.cpp
  src/ll_path_publisher_node.cpp
  src/ll_path_follower_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_cmake_auto
  cv_bridge
)

include_directories(${cv_bridge_INCLUDE_DIRS})
link_directories(${cv_bridge_LIBRARY_DIRS})

target_link_libraries(lane_line_publisher
  ${cv_bridge_LIBRARIES}
)

ament_auto_package(
  INSTALL_TO_SHARE
)