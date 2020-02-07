cmake_minimum_required(VERSION 2.8.3)
project(doogie_perception)

add_compile_options(-std=c++11)

set(DOOGIE_CORE_DEPS 
  roscpp
  doogie_core
  doogie_msgs
)

find_package(catkin REQUIRED COMPONENTS ${DOOGIE_CORE_DEPS})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${DOOGIE_CORE_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(doogie_perception_node 
  src/doogie_perception.cpp
  src/doogie_perception_node.cpp
)
target_link_libraries(doogie_perception_node
  ${catkin_LIBRARIES}
)
