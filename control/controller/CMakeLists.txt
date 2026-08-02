cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(controller_node SHARED
  src/controller_node.cpp
)

target_compile_features(controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(controller_node PRIVATE "CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package()
