cmake_minimum_required(VERSION 3.0.2)
project(polygon_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Polygon3D.msg
  MapManager.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



