cmake_minimum_required(VERSION 3.5)
project(simulator)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY  launch world models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/steered_to_twist.py
    scripts/convert_sim_to_vectornav_pose.py
    scripts/convert_sim_to_vectornav_velocity_body.py
    scripts/convert_image_encoding.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
