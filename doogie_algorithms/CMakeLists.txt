cmake_minimum_required(VERSION 2.8.3)
project(doogie_algorithms)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  COMPONENTS
    doogie_msgs
    doogie_navigation
)

set(DOOGIE_ALGORITHMS_INCLUDE_DIRS
  right_hand_plugin/include
)

catkin_package(
  INCLUDE_DIRS 
    ${DOOGIE_ALGORITHMS_INCLUDE_DIRS}
  CATKIN_DEPENDS 
    doogie_msgs
    doogie_navigation
# LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(matrix_handle src/matrix_handle.cpp)
# target_link_libraries(matrix_handle ${catkin_LIBRARIES})