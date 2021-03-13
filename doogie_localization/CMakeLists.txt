cmake_minimum_required(VERSION 3.1.3)
project(doogie_localization)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  angles
  nav_msgs 
  pluginlib
  roscpp 
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES base_localization odom_controller
  CATKIN_DEPENDS roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(base_localization
  src/base_localization.cpp
)
target_link_libraries(base_localization
  ${catkin_LIBRARIES}
)

add_library(odom_controller
  src/odom_controller.cpp
)
target_link_libraries(odom_controller
  base_localization ${catkin_LIBRARIES}
)
