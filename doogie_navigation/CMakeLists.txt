cmake_minimum_required(VERSION 3.1.3)
project(doogie_navigation)

set(CMAKE_CXX_STANDARD 11)

set(CATKIN_BUILD_DEPS
  rospy
  roscpp
  doogie_core
  doogie_control
  doogie_msgs
  doogie_algorithms
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

add_library(move_base
  src/move_base.cpp
  src/move_base_params.cpp
)
target_link_libraries(move_base
  ${catkin_LIBRARIES}
)

add_library(move_in_maze
  src/move_in_maze.cpp
  src/local_cell.cpp
  src/maze_goal.cpp
)
target_link_libraries(move_in_maze
  move_base
  ${catkin_LIBRARIES}
)

add_executable(move_base_node 
  src/move_base_node.cpp
)
add_dependencies(move_base_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_node move_base ${catkin_LIBRARIES})

add_executable(move_in_maze_node 
  src/move_in_maze_node.cpp
)
add_dependencies(move_in_maze_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_in_maze_node move_in_maze ${catkin_LIBRARIES})

add_executable(maze_solver_node src/maze_solver_node.cpp)
add_dependencies(maze_solver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_solver_node ${catkin_LIBRARIES})


if (CATKIN_ENABLE_TESTING)
catkin_add_gtest(direction_operations tests/directions_operation.cpp)
add_dependencies(direction_operations ${catkin_EXPORTED_TARGETS})
target_link_libraries(direction_operations ${catkin_LIBRARIES})

endif()

add_executable(fake_planner tests/fake_planner.cpp)
add_dependencies(fake_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_planner ${catkin_LIBRARIES})
