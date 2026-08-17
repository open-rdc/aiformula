cmake_minimum_required(VERSION 3.8)
project(main_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(main_exec
        src/main.cpp
        )
target_compile_features(main_exec PUBLIC c_std_99 cxx_std_17)

ament_auto_package(INSTALL_TO_SHARE launch config)
