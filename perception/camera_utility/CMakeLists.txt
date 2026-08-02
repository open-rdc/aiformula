cmake_minimum_required(VERSION 3.8)
project(camera_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

ament_auto_add_library(camera_utility SHARED
  src/camera_intrinsics.cpp
  src/ground_conversion.cpp
  src/camera_parameters.cpp
)
target_compile_features(camera_utility PUBLIC c_std_99 cxx_std_17)
target_include_directories(camera_utility PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_utility ${OpenCV_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_utility test/test_camera_utility.cpp)
  target_link_libraries(test_camera_utility camera_utility)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(OpenCV)

ament_auto_package()
