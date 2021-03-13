cmake_minimum_required(VERSION 2.8.3)
project(doogie_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  control_toolbox
)

catkin_package(
	INCLUDE_DIRS
		include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
    src/pid_controller.cpp
    src/diff_drive_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})