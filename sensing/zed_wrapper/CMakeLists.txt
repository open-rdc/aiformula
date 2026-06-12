cmake_minimum_required(VERSION 3.8)
project(zed_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ZED SDK is a system library — not a ROS package
set(ZED_DIR "/usr/local/zed/")
find_package(zed REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

ament_auto_add_library(zed_wrapper_node SHARED
  src/zed_wrapper_node.cpp
)
target_compile_features(zed_wrapper_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(zed_wrapper_node PRIVATE "ZED_WRAPPER_BUILDING_LIBRARY")
target_include_directories(zed_wrapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(zed_wrapper_node
  ${ZED_LIBRARIES}
  ${CUDA_DEP_LIBRARIES_ZED}
)

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
