cmake_minimum_required(VERSION 3.8)
project(vectormap_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

ament_auto_add_library(vectormap_server_node SHARED
  src/osm_parser.cpp
  src/vectormap_server_node.cpp
  src/vectormap_visualizer.cpp
)
target_compile_features(vectormap_server_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(vectormap_server_node PRIVATE "VECTORMAP_SERVER_BUILDING_LIBRARY")
target_link_libraries(vectormap_server_node tinyxml2::tinyxml2)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(tinyxml2_vendor TinyXML2)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
