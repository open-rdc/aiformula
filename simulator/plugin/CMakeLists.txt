# plugin/CMakeLists.txt

add_library(ColorControl SHARED
  ColorControl.cc
)

add_library(LightControl SHARED
  LightControl.cc
)

target_include_directories(ColorControl PRIVATE
  .
)

target_include_directories(LightControl PRIVATE
  .
)


target_link_libraries(ColorControl
  ignition-gazebo6::core
)

target_link_libraries(LightControl
  ignition-gazebo6::core
)

install(TARGETS ColorControl LightControl
  DESTINATION lib
)
