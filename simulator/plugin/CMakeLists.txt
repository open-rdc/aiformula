find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

add_library(ObstacleControl SHARED
  ObstacleControl.cc
)

target_include_directories(ObstacleControl PRIVATE
  .
  ${PROJ_INCLUDE_DIRS}
)

target_link_libraries(ObstacleControl
  PRIVATE
    ignition-gazebo6::core
    ament_index_cpp::ament_index_cpp
    ${PROJ_LIBRARIES}
)

install(TARGETS ObstacleControl
  DESTINATION lib
)