cmake_minimum_required(VERSION 3.5)
project(gnss_navigation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)  
find_package(tf2_ros REQUIRED)  
find_package(tf2_geometry_msgs REQUIRED)
# Include directories for PROJ
find_path(PROJ_INCLUDE_DIR NAMES proj.h PATHS /usr/include)
find_library(PROJ_LIBRARY NAMES proj PATHS /usr/lib/x86_64-linux-gnu)

# Include directories
include_directories(
  include
)

# Declare a C++ executable for gnss_path_publisher
add_executable(gnss_path_publisher src/gnss_path_publisher.cpp)
target_include_directories(gnss_path_publisher PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Declare a C++ executable for gnss_path_follower
add_executable(gnss_path_follower src/gnss_path_follower.cpp)
target_include_directories(gnss_path_follower PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Specify libraries to link a library or executable target against
target_link_libraries(gnss_path_publisher
  ${PROJ_LIBRARY}
)

target_link_libraries(gnss_path_follower
  ${PROJ_LIBRARY} 
)

# Additional target dependencies
ament_target_dependencies(gnss_path_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  Eigen3
  tf2  
  tf2_ros 
  tf2_geometry_msgs
)

ament_target_dependencies(gnss_path_follower
  rclcpp
  nav_msgs
  geometry_msgs
  Eigen3
  tf2 
  tf2_ros
  tf2_geometry_msgs  
)

ament_export_include_directories(
  include
)

# Install targets
install(TARGETS
  gnss_path_publisher
  gnss_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_package()

