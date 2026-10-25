cmake_minimum_required(VERSION 3.8)
project(topomap_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(TinyXML2 REQUIRED)

ament_auto_add_library(topomap_server_node SHARED
  src/topomap_server_node.cpp
)
target_compile_features(topomap_server_node PUBLIC cxx_std_17)
target_compile_definitions(topomap_server_node PRIVATE "TOPOMAP_SERVER_BUILDING_LIBRARY")
target_link_libraries(topomap_server_node tinyxml2::tinyxml2)

ament_export_dependencies(TinyXML2)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    config
)
