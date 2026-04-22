cmake_minimum_required(VERSION 3.0.2)
project(robot_visual_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  urdf
  tf
  pcl_ros
  pcl_conversions
  geometric_shapes
  moveit_core
  robot_state_publisher
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs urdf tf pcl_ros pcl_conversions geometric_shapes moveit_core robot_state_publisher
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_executable(visual_pointcloud_node src/visual_pointcloud_node.cpp)
target_link_libraries(visual_pointcloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

