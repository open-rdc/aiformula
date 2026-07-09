cmake_minimum_required(VERSION 3.8)
project(ekf_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(ekf_localizer_node SHARED
  src/ekf_localizer.cpp
  src/ekf_localizer_node.cpp
  src/map_odom_tf_node.cpp
  src/odom_tf_node.cpp
  src/mahalanobis.cpp
)
target_compile_features(ekf_localizer_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(ekf_localizer_node PRIVATE "EKF_LOCALIZER_BUILDING_LIBRARY")
target_include_directories(ekf_localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(ekf_localizer_node
  Eigen3::Eigen
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ekf_localizer
    test/test_mahalanobis.cpp
    test/test_delay_gate.cpp
    test/test_velocity_gate.cpp
    test/test_ekf_localizer_gate.cpp
  )
  target_link_libraries(test_ekf_localizer ekf_localizer_node)
endif()

ament_auto_package()
