cmake_minimum_required(VERSION 3.8)
project(main_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(socketcan_interface REQUIRED)
find_package(controller REQUIRED)
find_package(chassis_driver REQUIRED)
find_package(cybergear_interface REQUIRED)
find_package(gnssnav REQUIRED)
find_package(lane_line_publisher REQUIRED)

add_executable(main_exec
  src/main.cpp
)

ament_target_dependencies(main_exec
  rclcpp
  socketcan_interface
  controller
  chassis_driver
  cybergear_interface
  gnssnav
  lane_line_publisher
)

target_include_directories(main_exec
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME})

install(TARGETS
        main_exec
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()