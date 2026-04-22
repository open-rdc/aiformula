cmake_minimum_required(VERSION 3.8)
project(frenet_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(frenet_planner_node SHARED
  src/frenet_planner_node.cpp
  src/frenet_planner.cpp
  src/reference_curve.cpp
  src/risk_calculator.cpp
  src/obstacle_detect.cpp
)
target_compile_features(frenet_planner_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(frenet_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(frenet_planner_node Eigen3::Eigen)
target_compile_definitions(frenet_planner_node PRIVATE "FRENET_PLANNER_BUILDING_LIBRARY")

ament_export_dependencies(Eigen3)
ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})

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
