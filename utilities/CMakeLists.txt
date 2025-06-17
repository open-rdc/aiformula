cmake_minimum_required(VERSION 3.8)
project(utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

# PyTorch dependencies (required) - using local libtorch
set(TORCH_INCLUDE_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/libtorch/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/libtorch/include/torch/csrc/api/include"
)
set(TORCH_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libtorch/lib")

# Manually set torch libraries without problematic targets
set(TORCH_LIBRARIES 
    "${TORCH_LIB_DIR}/libtorch.so"
    "${TORCH_LIB_DIR}/libtorch_cpu.so"
    "${TORCH_LIB_DIR}/libc10.so"
)

include_directories(include)

add_library(utilities
  src/utils.cpp
  src/velplanner.cpp
)
target_compile_features(utilities PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(utilities PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(utilities PRIVATE
  ${TORCH_INCLUDE_DIRS})
ament_target_dependencies(
        utilities
        "rclcpp"
)

# Link PyTorch libraries directly
target_link_libraries(utilities ${TORCH_LIBRARIES})

# Set torch compile definitions to exclude problematic CUDA components
set_property(TARGET utilities PROPERTY CXX_STANDARD 17)
target_compile_definitions(utilities PRIVATE TORCH_DISABLE_NVTX=1)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(utilities PRIVATE "UTILITIES_BUILDING_LIBRARY")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
)
# Standard library exports only
ament_export_libraries(utilities)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS utilities
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  utilities
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
