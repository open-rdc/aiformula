cmake_minimum_required(VERSION 3.8)
project(traffic_signal_stop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(traffic_signal_stop_node SHARED
  src/traffic_signal_stop_node.cpp
)
target_compile_features(traffic_signal_stop_node PUBLIC cxx_std_17)
target_compile_definitions(traffic_signal_stop_node PRIVATE "TRAFFIC_SIGNAL_STOP_BUILDING_LIBRARY")

ament_auto_package()
