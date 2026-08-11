cmake_minimum_required(VERSION 3.10)
project(gazebo_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(PathPublisher SHARED
  PathPublisher.cc
)

target_include_directories(PathPublisher PRIVATE
  .
  ${PROJ_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(PathPublisher
  PRIVATE
    ignition-gazebo6::core
    ignition-transport11::core
    ignition-msgs8::core
    ament_index_cpp::ament_index_cpp
    ${PROJ_LIBRARIES}
)

add_library(FollowPath SHARED
  FollowPath.cc
)

target_include_directories(FollowPath PRIVATE
  .
  ${PROJ_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(FollowPath
  PRIVATE
    ignition-gazebo6::core
    ignition-transport11::core
    ignition-msgs8::core
    ament_index_cpp::ament_index_cpp
    ${PROJ_LIBRARIES}
)

install(TARGETS 
  PathPublisher 
  FollowPath
  DESTINATION lib
)