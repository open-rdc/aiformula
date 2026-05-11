cmake_minimum_required(VERSION 3.8)
project(object_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common filters segmentation search kdtree)

ament_auto_add_library(object_detector_node SHARED
  src/object_detector_node.cpp)
target_compile_features(object_detector_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(object_detector_node PRIVATE "OBJECT_DETECTOR_BUILDING_LIBRARY")
target_include_directories(object_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(object_detector_node ${PCL_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(PCL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
