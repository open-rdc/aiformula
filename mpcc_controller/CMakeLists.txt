cmake_minimum_required(VERSION 3.14)
project(mpcc_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# -----------------------------------------------------------------------
# BLASFEO / HPIPM (FetchContent — before compile options to avoid -Werror)
# -----------------------------------------------------------------------
include(FetchContent)

set(BLASFEO_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BLASFEO_TESTING  OFF CACHE BOOL "" FORCE)
if(NOT DEFINED TARGET)
  set(TARGET "GENERIC" CACHE STRING "BLASFEO target architecture")
endif()

FetchContent_Declare(blasfeo
  GIT_REPOSITORY https://github.com/giaf/blasfeo.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(blasfeo)

set(HPIPM_TESTING OFF CACHE BOOL "" FORCE)
if(NOT DEFINED HPIPM_TARGET)
  set(HPIPM_TARGET "GENERIC" CACHE STRING "HPIPM target architecture")
endif()
set(OLD_TARGET "${TARGET}")
set(TARGET "${HPIPM_TARGET}" CACHE STRING "" FORCE)

FetchContent_Declare(hpipm
  GIT_REPOSITORY https://github.com/giaf/hpipm.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(hpipm)
set(TARGET "${OLD_TARGET}" CACHE STRING "" FORCE)

# 自プロジェクトのソースのみ警告フラグを適用
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------
# MPCC コントローラプラグインライブラリ (先に宣言: motion_models が依存するため)
# -----------------------------------------------------------------------
ament_auto_add_library(mpcc_controller_plugin SHARED
  src/types.cpp
  src/integrator.cpp
  src/params.cpp
  src/cost.cpp
  src/constraints.cpp
  src/bounds.cpp
  src/spline.cpp
  src/hpipm_interface.cpp
  src/mpc.cpp
  src/mpcc_controller_plugin.cpp
)
target_compile_features(mpcc_controller_plugin PUBLIC cxx_std_17)
target_compile_definitions(mpcc_controller_plugin PRIVATE "MPCC_CONTROLLER_BUILDING_LIBRARY")
target_include_directories(mpcc_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpcc_controller_plugin hpipm blasfeo)
pluginlib_export_plugin_description_file(motion_control plugins.xml)

# -----------------------------------------------------------------------
# 運動モデルプラグインライブラリ (pluginlib 経由でのみロード: controller_plugin に依存しない逆方向)
# -----------------------------------------------------------------------
ament_auto_add_library(mpcc_motion_models SHARED
  src/motion_model.cpp
)
target_compile_features(mpcc_motion_models PUBLIC cxx_std_17)
target_include_directories(mpcc_motion_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpcc_motion_models mpcc_controller_plugin)
pluginlib_export_plugin_description_file(mpcc_controller motion_models.xml)

install(DIRECTORY include/ DESTINATION include)

ament_auto_package()
