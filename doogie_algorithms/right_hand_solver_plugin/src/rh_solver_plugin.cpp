#include "doogie_algorithms/rh_solver_plugin/rh_solver_plugin.hpp"

// register RHSolver as a BaseSolver implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(doogie_algorithms::RHSolverPlugin, doogie_algorithms::BaseSolver)

namespace doogie_algorithms{

void RHSolverPlugin::init(){
  maze_walls_sub_ = nh_.subscribe("maze_walls_matrix", 100, &RHSolverPlugin::mazeWallsCallback, this);
  doogie_position_sub_ = nh_.subscribe("doogie_position", 100, &RHSolverPlugin::doogiePositionCallback, this);
  ROS_INFO("Doing well");
}

void RHSolverPlugin::mazeWallsCallback(const doogie_msgs::MazeCellMultiArray& matrix_msg){
  *matrix_maze_ = matrix_msg;
}

void RHSolverPlugin::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  current_cell_ = matrix_handle_.GlobalToLocal(position_msg);
  doogie_handle_.setPosition(position_msg);
}

};