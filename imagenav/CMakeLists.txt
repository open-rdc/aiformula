cmake_minimum_required(VERSION 3.8)
project(imagenav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# auto find dependencies
find_package(ament_cmake_auto REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

# add_executable(imagenav_node
#   src/debug_camera.cpp
# )
ament_auto_add_library(imagenav
  src/imagenav_node.cpp
  src/line_detector.cpp
  src/obstacle_detector.cpp
)

# target_link_libraries(imagenav_node imagenav)
target_link_libraries(imagenav Eigen3::Eigen)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_cmake_auto
)

target_link_libraries(${PROJECT_NAME}
  # 他の依存ライブラリ
)

ament_auto_package(
  INSTALL_TO_SHARE
)