cmake_minimum_required(VERSION 3.0.2)
project(robot_pointcloud_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  tf
  urdf
  geometric_shapes
)

find_package(PCL REQUIRED)

set(EIGEN3_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package()

add_executable(robot_pointcloud_filter src/robot_pointcloud_filter.cpp)
target_link_libraries(robot_pointcloud_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS robot_pointcloud_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
