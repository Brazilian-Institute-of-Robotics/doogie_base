cmake_minimum_required(VERSION 2.8.3)
project(doogie_algorithms)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  COMPONENTS
    pluginlib
    doogie_core
)

find_package(Boost)


set(DOOGIE_ALGORITHMS_INCLUDE_DIRS
  right_hand_solver_plugin/include
)

catkin_package(
  INCLUDE_DIRS 
    include
    ${DOOGIE_ALGORITHMS_INCLUDE_DIRS}
  LIBRARIES 
    base_solver
  CATKIN_DEPENDS 
    pluginlib
    doogie_core
  DEPENDS
    Boost
  )

include_directories(
  include
  ${DOOGIE_ALGORITHMS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(base_solver
  src/base_solver.cpp
  src/solver_plugin_loader.cpp
)

add_subdirectory(right_hand_solver_plugin)