cmake_minimum_required(VERSION 3.10)
project(polygon_mapping_cuda LANGUAGES CXX CUDA)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  tf
)

# OpenCV
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV version: ${OpenCV_VERSION} from ${OpenCV_DIR}")

# CUDA
find_package(CUDA REQUIRED)
message(STATUS "Using CUDA version: ${CUDA_VERSION}")

# PCL
find_package(PCL REQUIRED)
message(STATUS "Using PCL version: ${PCL_VERSION}")

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport tf
)

# include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/opencv4        # <-- 确保包含 OpenCV CUDA 头文件
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

# C++ / CUDA 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_HOST_COMPILER /usr/bin/gcc)

# 添加 CUDA 库
add_library(normal_cuda STATIC src/normal_estimation_cuda.cu)
add_library(diffusion_cuda STATIC src/anisotropic_diffusion_cuda.cu)
add_library(sharpen_cuda STATIC src/sharpen_cuda.cu)
add_library(pointcloud_cuda STATIC src/pointcloud_cuda.cu)

# 可执行文件
add_executable(image_subscriber_node src/image_subscriber.cpp)

# 链接库
target_link_libraries(image_subscriber_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  opencv_cudaimgproc          # <-- 添加 OpenCV CUDA 库
  opencv_cudaarithm
  normal_cuda
  diffusion_cuda
  sharpen_cuda
  pointcloud_cuda
  ${CUDA_LIBRARIES}
  ${PCL_LIBRARIES}
)

# CUDA separable compilation
set_target_properties(normal_cuda diffusion_cuda sharpen_cuda pointcloud_cuda image_subscriber_node PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

# CUDA 架构
set_target_properties(normal_cuda diffusion_cuda sharpen_cuda pointcloud_cuda image_subscriber_node PROPERTIES
  CUDA_ARCHITECTURES 61
)
