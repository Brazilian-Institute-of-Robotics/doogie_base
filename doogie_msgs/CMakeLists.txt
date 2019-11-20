cmake_minimum_required(VERSION 2.8.3)
project(doogie_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  MazeCell.msg
  MazeCellMultiArray.msg
  UpdateMatrix.msg
  WallDistances.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs sensor_msgs std_msgs
)
