cmake_minimum_required(VERSION 2.8.3)
project(doogie_core)

add_compile_options(-std=c++11)

set(DOOGIE_CORE_LIB_NAME doogie_core)
set(DOOGIE_CORE_DEPS 
  roscpp
  actionlib
  doogie_msgs
  pluginlib
)

find_package(catkin REQUIRED COMPONENTS ${DOOGIE_CORE_DEPS})
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${DOOGIE_CORE_DEPS}
  LIBRARIES ${DOOGIE_CORE_LIB_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${DOOGIE_CORE_LIB_NAME}
  src/base_solver.cpp
  src/maze_matrix_handle.cpp
  src/mouse_handle.cpp
  src/solver_plugin_loader.cpp
)
target_link_libraries(${DOOGIE_CORE_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${DOOGIE_SOLVER_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(TARGETS ${DOOGIE_SOLVER_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*~" EXCLUDE
)
