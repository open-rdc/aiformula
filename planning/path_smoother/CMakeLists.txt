cmake_minimum_required(VERSION 3.8)
project(path_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(smoother SHARED
  src/smoother.cpp
)
target_compile_features(smoother PUBLIC cxx_std_17)
target_compile_definitions(smoother PRIVATE "PATH_SMOOTHER_BUILDING_LIBRARY")

ament_auto_add_library(path_smoother_node SHARED
  src/path_smoother_node.cpp
)
target_compile_features(path_smoother_node PUBLIC cxx_std_17)
target_compile_definitions(path_smoother_node PRIVATE "PATH_SMOOTHER_BUILDING_LIBRARY")
target_link_libraries(path_smoother_node smoother)

ament_auto_package()
