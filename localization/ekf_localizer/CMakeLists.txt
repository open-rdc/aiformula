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
  src/velocity_gate.cpp
)
target_compile_features(ekf_localizer_node PUBLIC cxx_std_17)
target_compile_definitions(ekf_localizer_node PRIVATE "EKF_LOCALIZER_BUILDING_LIBRARY")
target_link_libraries(ekf_localizer_node
  Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_delay_gate test/test_delay_gate.cpp)
  target_link_libraries(test_delay_gate ekf_localizer_node)
  ament_add_gtest(test_velocity_gate test/test_velocity_gate.cpp)
  target_link_libraries(test_velocity_gate ekf_localizer_node)
  ament_add_gtest(test_ekf_localizer_covariance_floor test/test_ekf_localizer_covariance_floor.cpp)
  target_link_libraries(test_ekf_localizer_covariance_floor ekf_localizer_node)
  ament_add_gtest(test_ekf_localizer_gate_timeout test/test_ekf_localizer_gate_timeout.cpp)
  target_link_libraries(test_ekf_localizer_gate_timeout ekf_localizer_node)
  ament_add_gtest(test_ekf_localizer_input_contract test/test_ekf_localizer_input_contract.cpp)
  target_link_libraries(test_ekf_localizer_input_contract ekf_localizer_node)
endif()

ament_auto_package()
