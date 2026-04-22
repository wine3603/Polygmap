cmake_minimum_required(VERSION 3.0.2)
project(depth_to_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_to_cloud_node src/depth_to_cloud_node.cpp)
target_link_libraries(depth_to_cloud_node ${catkin_LIBRARIES})

