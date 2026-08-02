cmake_minimum_required(VERSION 3.8)
project(lane_line_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_library(lane_line_publisher_node SHARED
  src/ground_projection.cpp
  src/lane_line_resampling.cpp
  src/line_components.cpp
  src/lane_line_publisher_node.cpp
  src/vectormap_projection.cpp
  src/vectormap_visualizer_node.cpp
)
target_compile_features(lane_line_publisher_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(lane_line_publisher_node PRIVATE "LANE_LINE_PUBLISHER_BUILDING_LIBRARY")
target_include_directories(lane_line_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lane_line_publisher_node
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lane_line_resampling test/test_lane_line_resampling.cpp)
  target_link_libraries(test_lane_line_resampling lane_line_publisher_node)
  ament_add_gtest(test_ground_projection test/test_ground_projection.cpp)
  target_link_libraries(test_ground_projection lane_line_publisher_node)
  ament_add_gtest(test_vectormap_projection test/test_vectormap_projection.cpp)
  target_link_libraries(test_vectormap_projection lane_line_publisher_node)
  ament_add_gtest(test_line_components test/test_line_components.cpp)
  target_link_libraries(test_line_components lane_line_publisher_node)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(Eigen3 OpenCV)

ament_auto_package()
