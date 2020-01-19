set(DOOGIE_CORE_LIB_NAME doogie_core)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${DOOGIE_CORE_LIB_NAME}
  src/base_solver.cpp
  src/matrix_handle.cpp
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