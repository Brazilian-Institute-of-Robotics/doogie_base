cmake_minimum_required(VERSION 2.8.3)
project(doogie_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
  actionlib_msgs
)

add_message_files(
  FILES
    DoogiePosition.msg
    DoogieOrientation.msg
    DoogiePose.msg
    MazeCell.msg
    MazeCellMultiArray.msg
    WallDistances.msg
)

add_action_files(
  DIRECTORY 
    action
  FILES 
    DoogieMove.action
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    nav_msgs 
    sensor_msgs 
    std_msgs
)
