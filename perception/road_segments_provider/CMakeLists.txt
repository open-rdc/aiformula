cmake_minimum_required(VERSION 3.8)
project(road_segments_provider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

ament_auto_add_library(road_segments_provider_node SHARED
  src/ipm.cpp
  src/road_segments_provider_node.cpp
)
target_compile_features(road_segments_provider_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(road_segments_provider_node PRIVATE "ROAD_SEGMENTS_PROVIDER_BUILDING_LIBRARY")
target_include_directories(road_segments_provider_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(road_segments_provider_node ${OpenCV_LIBS})

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
