cmake_minimum_required(VERSION 3.8)
project(vector_e2e_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

# 純関数層はTensorRT非依存。GPUの無い環境でもビルド・テストできるようTensorRT検出より前に置く
ament_auto_add_library(vector_e2e_path_builder SHARED
  src/branch_counter.cpp
  src/path_builder.cpp
)
target_compile_features(vector_e2e_path_builder PUBLIC cxx_std_17)
target_compile_definitions(vector_e2e_path_builder PRIVATE "VECTOR_E2E_PLANNER_BUILDING_LIBRARY")
target_include_directories(vector_e2e_path_builder PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vector_e2e_path_builder ${OpenCV_LIBRARIES})

find_package(CUDA QUIET)
# TensorRT は JetPack では /usr/include/aarch64-linux-gnu、開発PCでは CUDA 同梱位置に入る
find_path(NVINFER_INCLUDE_DIR NvInfer.h
  HINTS ${CUDA_INCLUDE_DIRS} /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu)
find_library(NVINFER_LIBRARY nvinfer
  HINTS ${CUDA_LIBRARIES} /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

# デプロイ成果物だけをインストールする。.pt と学習設定は学習側で管理する
install(DIRECTORY weights/
  DESTINATION share/${PROJECT_NAME}/weights
)

if(CUDA_FOUND AND NVINFER_INCLUDE_DIR AND NVINFER_LIBRARY)
  ament_export_definitions("ENABLE_VECTOR_E2E_PLANNER")

  ament_auto_add_library(vector_e2e_planner_node SHARED
    src/vector_e2e_planner_node.cpp
    src/vector_e2e_tensorrt.cpp
  )
  target_compile_features(vector_e2e_planner_node PUBLIC cxx_std_17)
  target_compile_definitions(vector_e2e_planner_node PRIVATE "VECTOR_E2E_PLANNER_BUILDING_LIBRARY")
  target_include_directories(vector_e2e_planner_node PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${NVINFER_INCLUDE_DIR}
  )
  target_link_libraries(vector_e2e_planner_node
    vector_e2e_path_builder
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${NVINFER_LIBRARY}
  )
else()
  message(WARNING "TensorRT/CUDA not found — skipping vector_e2e_planner node build")
endif()

ament_export_dependencies(OpenCV)

ament_auto_package()
