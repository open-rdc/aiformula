cmake_minimum_required(VERSION 3.5)
project(odometry_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)


# wheel_odometry
file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/wheel_odometry/*.cpp")
add_executable(wheel_odometry
${CMAKE_CURRENT_SOURCE_DIR}/src/common.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/wheel_odometry/main.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/wheel_odometry/wheel_odometry.cpp
)
target_include_directories(wheel_odometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(wheel_odometry
    rclcpp
    tf2
    tf2_ros
    nav_msgs
    socketcan_interface_msg
)
install(TARGETS wheel_odometry
    DESTINATION lib/${PROJECT_NAME}
)

# gyro_odometry
add_executable(gyro_odometry
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gyro_odometry/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gyro_odometry/gyro_odometry.cpp
)
target_include_directories(gyro_odometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(gyro_odometry
    rclcpp
    tf2_ros
    sensor_msgs
    nav_msgs
    socketcan_interface_msg
)
install(TARGETS gyro_odometry
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  socketcan_interface_msg
)

ament_package()
