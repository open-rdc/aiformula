cmake_minimum_required(VERSION 3.5)
project(gnssnav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKFE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_path(PROJ_INCLUDE_DIR NAMES proj.h PATHS /usr/include)
find_library(PROJ_LIBRARY NAMES proj PATHS /usr/lib/x86_64-linux-gnu)

include_directories(/usr/include/eigen3)

add_library(navigation_node
  src/follower_node.cpp
  src/path_publisher_node.cpp)
target_include_directories(navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  navigation_node
  ament_index_cpp
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(navigation_node ${PROJ_LIBRARY})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(navigation_node PRIVATE "GNSSNAV_BUILDING_LIBRARY")


ament_export_include_directories(
  include
)
ament_export_libraries(
  navigation_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
        ament_index_cpp
        rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS navigation_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(DIRECTORY config
#   DESTINATION share/main_executor)

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
