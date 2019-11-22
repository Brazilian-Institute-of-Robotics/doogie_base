cmake_minimum_required(VERSION 2.8.3)
project(doogie_navigation)

add_compile_options(-std=c++11)

set(CATKIN_BUILD_DEPS
  rospy
  roscpp
  std_msgs
  doogie_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  actionlib
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})
find_package(Boost REQUIRED COMPONENTS system)
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_BUILD_DEPS}
  DEPENDS Boost
  LIBRARIES matrix_handle
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(matrix_handle src/matrix_handle.cpp)
target_link_libraries(matrix_handle ${catkin_LIBRARIES})

install(TARGETS matrix_handle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*~" EXCLUDE
)

add_library(move_base src/move_base.cpp)
target_link_libraries(move_base 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(move_base_node src/move_base_node.cpp)
target_link_libraries(move_base_node
  move_base
  ${catkin_LIBRARIES}
)