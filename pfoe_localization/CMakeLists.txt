cmake_minimum_required(VERSION 3.8)
project(pfoe_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(steered_drive_msg REQUIRED)

add_library(${PROJECT_NAME}
  src/pfoe_node.cpp
  src/ParticleFilter.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  ament_index_cpp
  steered_drive_msg
)

add_executable(pfoe_test src/main_test.cpp)
target_link_libraries(pfoe_test ${PROJECT_NAME})   # ↑のlibraryをリンク
ament_target_dependencies(pfoe_test rclcpp)
install(TARGETS pfoe_test
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs ament_index_cpp)

install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
