cmake_minimum_required(VERSION 3.10)
project(gazebo_plugins)

# C++17 標準を要求
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 必要なパッケージの探索
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

# Ignition Gazebo 関連パッケージ
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# ターゲット 1: PathPublisher プラグイン
# ==============================================================================
add_library(PathPublisher SHARED
  PathPublisher.cc
)

target_include_directories(PathPublisher PRIVATE
  .
  ${PROJ_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(PathPublisher
  PRIVATE
    ignition-gazebo6::core
    ignition-transport11::core
    ignition-msgs8::core
    ament_index_cpp::ament_index_cpp
    ${PROJ_LIBRARIES}
)

# ==============================================================================
# ターゲット 2: FollowPath プラグイン
# ==============================================================================
add_library(FollowPath SHARED
  FollowPath.cc
)

target_include_directories(FollowPath PRIVATE
  .
  ${PROJ_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(FollowPath
  PRIVATE
    ignition-gazebo6::core
    ignition-transport11::core
    ignition-msgs8::core
    ament_index_cpp::ament_index_cpp
    ${PROJ_LIBRARIES}
)

# ==============================================================================
# インストール設定（両方のライブラリを同時に指定）
# ==============================================================================
install(TARGETS 
  PathPublisher 
  FollowPath
  DESTINATION lib
)