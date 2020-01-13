cmake_minimum_required(VERSION 2.8.3)
project(doogie_algorithms)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  COMPONENTS
    doogie_msgs
    pluginlib
)

find_package(Boost REQUIRED COMPONENTS filesystem system)


set(DOOGIE_ALGORITHMS_INCLUDE_DIRS
  core/include
  right_hand_solver_plugin/include
)

catkin_package(
  INCLUDE_DIRS 
    ${DOOGIE_ALGORITHMS_INCLUDE_DIRS}
  CATKIN_DEPENDS 
    doogie_msgs
    pluginlib
  LIBRARIES
    doogie_core
    rh_solver_plugin
  DEPENDS
    Boost
  )

include_directories(
  ${DOOGIE_ALGORITHMS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory(right_hand_solver_plugin)
add_subdirectory(core)