cmake_minimum_required(VERSION 2.8.3)
project(doogie_core)

add_compile_options(-std=c++11)

set(DOOGIE_CORE_LIB_NAME doogie_core)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    actionlib
    doogie_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    actionlib
    doogie_msgs
  LIBRARIES
    doogie_core
  DEPENDS
    Boost
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${DOOGIE_CORE_LIB_NAME}
  src/base_solver.cpp
  src/matrix_handle.cpp
  src/mouse_handle.cpp
  src/solver_plugin_loader.cpp
)

target_link_libraries(${DOOGIE_CORE_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(${DOOGIE_SOLVER_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(
  TARGETS 
    ${DOOGIE_SOLVER_LIB_NAME}
  LIBRARY DESTINATION 
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION 
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)