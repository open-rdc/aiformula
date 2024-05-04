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
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${PROJ_INCLUDE_DIR}
  ${tf2_INCLUDE_DIRS} 
  ${tf2_ros_INCLUDE_DIRS}  
)

# Declare a C++ executable for gnss_path_publisher
add_executable(gnss_path_publisher gnss_navigation/gnss_path_publisher.cpp)

# Declare a C++ executable for gnss_path_follower
add_executable(gnss_path_follower gnss_navigation/gnss_path_follower.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(gnss_path_publisher
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PROJ_LIBRARY}
  ${tf2_LIBRARIES} 
  ${tf2_ros_LIBRARIES} 
)

target_link_libraries(gnss_path_follower
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PROJ_LIBRARY}
  ${tf2_LIBRARIES}  
  ${tf2_ros_LIBRARIES}  
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

# Install targets
install(TARGETS
  gnss_path_publisher
  gnss_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_package()

