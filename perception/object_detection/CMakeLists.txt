cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(CUDA QUIET)
# TensorRT は JetPack では /usr/include/aarch64-linux-gnu、開発PCでは CUDA 同梱位置に入る
find_path(NVINFER_INCLUDE_DIR NvInfer.h
  HINTS ${CUDA_INCLUDE_DIRS} /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu)
find_library(NVINFER_LIBRARY nvinfer
  HINTS ${CUDA_LIBRARIES} /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

# パイロンマスクの描画は TensorRT に依存しないので、TensorRT の無い PC でもビルドとテストができるように先に作る
add_library(pylon_mask STATIC src/pylon_mask.cpp)
target_compile_features(pylon_mask PUBLIC cxx_std_17)
set_target_properties(pylon_mask PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pylon_mask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(pylon_mask ${OpenCV_LIBRARIES})

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pylon_mask.cpp)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pylon_mask test/test_pylon_mask.cpp)
  target_link_libraries(test_pylon_mask pylon_mask)
endif()

# デプロイ成果物だけをインストールする。.pth と学習設定は学習側で管理する。
install(DIRECTORY weights/
  DESTINATION share/${PROJECT_NAME}/weights
  PATTERN "*.pth" EXCLUDE
)
if(NOT CUDA_FOUND OR NOT NVINFER_INCLUDE_DIR OR NOT NVINFER_LIBRARY)
  message(WARNING "TensorRT/CUDA not found — skipping object_detection build")
  ament_auto_package()
  return()
endif()

ament_export_definitions("ENABLE_TENSORRT")

ament_auto_add_library(pylon_detector_node SHARED
  src/pylon_detector_node.cpp
  src/yolox_tensorrt.cpp
)
target_compile_features(pylon_detector_node PUBLIC cxx_std_17)
target_compile_definitions(pylon_detector_node PRIVATE "OBJECT_DETECTION_BUILDING_LIBRARY")
target_include_directories(pylon_detector_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${NVINFER_INCLUDE_DIR}
)
target_link_libraries(pylon_detector_node
  pylon_mask
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${NVINFER_LIBRARY}
)

ament_export_dependencies(OpenCV)

ament_auto_package()
