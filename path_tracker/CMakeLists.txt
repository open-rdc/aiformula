cmake_minimum_required(VERSION 3.8)
project(path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pure_pursuit_node SHARED
  src/pure_pursuit_node.cpp
)
target_compile_features(pure_pursuit_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(pure_pursuit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pure_pursuit_node PRIVATE "PATH_TRACKER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

find_package(nav_msgs REQUIRED)
find_package(utilities REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)


add_executable(mpc_node src/mpc_node.cpp)
target_include_directories(mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc_node Eigen3::Eigen)
ament_target_dependencies(mpc_node
  rclcpp
  geometry_msgs
  nav_msgs
  utilities
  tf2
  tf2_geometry_msgs
  rclcpp_components
)

install(TARGETS mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
