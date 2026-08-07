cmake_minimum_required(VERSION 3.8)
project(vectormap_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(TinyXML2 REQUIRED)

ament_auto_add_library(vectormap_server_node SHARED
  src/osm_parser.cpp
  src/vectormap_server_node.cpp
  src/vectormap_visualizer.cpp
)
target_compile_features(vectormap_server_node PUBLIC cxx_std_17)
target_compile_definitions(vectormap_server_node PRIVATE "VECTORMAP_SERVER_BUILDING_LIBRARY")
target_link_libraries(vectormap_server_node tinyxml2::tinyxml2)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_osm_parser test/test_osm_parser.cpp)
  target_link_libraries(test_osm_parser vectormap_server_node)
endif()

ament_export_dependencies(TinyXML2)

ament_auto_package(
  INSTALL_TO_SHARE
    config
)
