cmake_minimum_required(VERSION 2.8.3)
project(doogie_navigation)

add_compile_options(-std=c++11)

set(CATKIN_BUILD_DEPS
  roscpp
  doogie_core
  doogie_msgs
  doogie_algorithms
  nav_msgs
  geometry_msgs
  tf
  actionlib
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_BUILD_DEPS} 
  DEPENDS Boost   
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(move_base_node 
  src/move_base_node.cpp
  src/move_base.cpp
)
add_dependencies(move_base_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_node ${catkin_LIBRARIES})

add_executable(maze_solver_node src/maze_solver_node.cpp)
add_dependencies(maze_solver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_solver_node ${catkin_LIBRARIES})
