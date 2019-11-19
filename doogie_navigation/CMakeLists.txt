cmake_minimum_required(VERSION 2.8.3)
project(doogie_navigation)

find_package(catkin REQUIRED COMPONENTS
rospy
std_msgs
message_generation
doogie_msgs
sensor_msgs)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  doogie_msgs
  sensor_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)