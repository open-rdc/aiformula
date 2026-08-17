cmake_minimum_required(VERSION 3.8)
project(pose_estimater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(pose_estimater_node SHARED
  src/particle_filter.cpp
  src/pose_estimater_node.cpp
)
target_compile_features(pose_estimater_node PUBLIC cxx_std_17)
target_compile_definitions(pose_estimater_node PRIVATE "POSE_ESTIMATER_BUILDING_LIBRARY")
target_link_libraries(pose_estimater_node
  Eigen3::Eigen
)

ament_auto_package()
