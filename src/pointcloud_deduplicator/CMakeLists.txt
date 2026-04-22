cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_deduplicator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(deduplicator_node src/deduplicator_node.cpp)

target_link_libraries(deduplicator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(deduplicator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

