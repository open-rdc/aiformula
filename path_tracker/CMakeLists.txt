cmake_minimum_required(VERSION 3.8)
project(path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. ROS2 パッケージの検出
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# 2. CasADi の検出
find_package(casadi REQUIRED)

# もし find_package で変数がセットされない場合の保険（直接ファイルを探す）
find_library(CASADI_LIBRARY_FILE NAMES casadi libcasadi PATHS /usr/local/lib /usr/lib)
if(NOT CASADI_LIBRARIES)
  set(CASADI_LIBRARIES ${CASADI_LIBRARY_FILE})
endif()

message(STATUS "CasADi Include: ${CASADI_INCLUDE_DIRS}")
message(STATUS "CasADi Lib: ${CASADI_LIBRARIES}")

# グローバルなインクルード設定
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}  # 大文字に修正
)

# 3. casadi_solver ライブラリのビルド
add_library(casadi_solver STATIC src/casadi_solver.cpp)

target_link_libraries(casadi_solver 
  ${CASADI_LIBRARIES}  # 大文字に修正
)

# 4. mpc_node のビルド
add_executable(mpc_node src/mpc_node.cpp)

target_include_directories(mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CASADI_INCLUDE_DIRS} # 大文字に修正
)

target_link_libraries(mpc_node 
  Eigen3::Eigen
  casadi_solver
  ${CASADI_LIBRARIES} # 大文字に修正（ここが最重要！）
)

ament_target_dependencies(mpc_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  rclcpp_components
)

# 5. インストール設定
install(TARGETS mpc_node casadi_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
  scripts/path_plotter.py
  scripts/estimate_params.py
  scripts/compare_mpc.py
  scripts/evaluate_model.py
  scripts/estimate_gains.py
  scripts/extract_path_from_bag.py
  scripts/publish_path_from_csv.py
  scripts/evaluate_tracking.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()