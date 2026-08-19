cmake_minimum_required(VERSION 3.8)
project(road_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(CUDA QUIET)
# TensorRT は JetPack では /usr/include/aarch64-linux-gnu、開発PCでは x86_64 側に入る
find_path(NVINFER_INCLUDE_DIR NvInfer.h
  HINTS ${CUDA_INCLUDE_DIRS} /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu)
find_library(NVINFER_LIBRARY nvinfer
  HINTS ${CUDA_LIBRARIES} /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

# engine は実行時に必要なのでインストールする。
# .pt(150MB) と .onnx(90MB) は engine を作るための中間成果物で実行時には不要なため除外する
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data
  PATTERN "*.pt" EXCLUDE
  PATTERN "*.onnx" EXCLUDE)

if(NOT CUDA_FOUND OR NOT NVINFER_INCLUDE_DIR OR NOT NVINFER_LIBRARY)
  message(WARNING "TensorRT/CUDA not found — skipping road_detector build")
  ament_auto_package()
  return()
endif()

ament_export_definitions("ENABLE_ROAD_DETECTOR")

ament_auto_add_library(road_detector_node SHARED
  src/letterbox.cpp
  src/lane_segmenter.cpp
  src/road_detector_node.cpp
)
target_compile_features(road_detector_node PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(road_detector_node PRIVATE "ROAD_DETECTOR_BUILDING_LIBRARY")
target_include_directories(road_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${NVINFER_INCLUDE_DIR}
)
target_link_libraries(road_detector_node
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${NVINFER_LIBRARY}
)

install(DIRECTORY include/ DESTINATION include)
ament_export_dependencies(OpenCV)

ament_auto_package()
