cmake_minimum_required(VERSION 3.8)
project(vectormap_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_library(vectormap_localization_node SHARED
  src/ekf_localizer.cpp
  src/icp_matching.cpp
  src/lane_pixel_to_point.cpp
  src/map_odom_tf_node.cpp
  src/odom_tf_node.cpp
  src/vectormap_localization_node.cpp
)
target_compile_features(vectormap_localization_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(vectormap_localization_node PRIVATE "VECTORMAP_LOCALIZATION_BUILDING_LIBRARY")
target_include_directories(vectormap_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(vectormap_localization_node
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(Eigen3 OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
