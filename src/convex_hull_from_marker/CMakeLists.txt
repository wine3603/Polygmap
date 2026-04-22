cmake_minimum_required(VERSION 3.0.2)
project(convex_hull_from_marker)

## 使用 C++14
add_compile_options(-std=c++14)

## 查找 catkin 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

## 查找其他依赖
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin 相关配置 ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###################################
## 编译可执行文件 ##
###################################

add_executable(convex_hull_dense_filler src/convex_hull_dense_filler.cpp)

target_link_libraries(convex_hull_dense_filler
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
