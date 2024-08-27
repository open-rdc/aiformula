cmake_minimum_required(VERSION 3.8)
project(socketcan_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(socketcan_interface_msg REQUIRED)

include_directories(include)

add_library(socketcan_interface_node
        src/socketcan_interface_node.cpp
        )

target_compile_definitions(socketcan_interface_node
        PRIVATE "SOCKETCAN_INTERFACE_LIBRARY"
        )
target_compile_options(socketcan_interface_node PUBLIC -Wall)

ament_target_dependencies(socketcan_interface_node
        ament_index_cpp
        rclcpp
        socketcan_interface_msg
        )

target_include_directories(socketcan_interface_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

if(NOT WIN32)
  ament_environment_hooks(
          "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        ament_index_cpp
        rclcpp
        socketcan_interface_msg
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS socketcan_interface_node
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

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

ament_package()
