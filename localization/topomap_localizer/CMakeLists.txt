cmake_minimum_required(VERSION 3.8)
project(topomap_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(topomap_localizer_node SHARED
  src/topomap_localizer_node.cpp
)
target_compile_features(topomap_localizer_node PUBLIC cxx_std_17)
target_compile_definitions(topomap_localizer_node PRIVATE "TOPOMAP_LOCALIZER_BUILDING_LIBRARY")

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
