#include "doogie_algorithms/rh_solver_plugin/rh_solver_plugin.hpp"

// register RHSolver as a BaseSolver implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(doogie_algorithms::RHSolverPlugin, doogie_core::BaseSolver)

namespace doogie_algorithms{

RHSolverPlugin::RHSolverPlugin(){
  doogie_position_sub_ = nh_.subscribe("doogie_position", 100, &RHSolverPlugin::doogiePositionCallback, this);
}

bool RHSolverPlugin::makePlan(){
  if (!isWallRight()){
    goal_.direction = doogie_msgs::DoogieMoveGoal::RIGHT;
    return true;
  }

  else if (!isWallFront()){
      goal_.direction = doogie_msgs::DoogieMoveGoal::FRONT;
      return true;
  }

  else if (!isWallLeft()){
      goal_.direction = doogie_msgs::DoogieMoveGoal::LEFT;
      return true;
  }

  else{
    ROS_ERROR("Could not make a plan. There is no path robot can move");
    return false;
  }
}

bool RHSolverPlugin::move(){
  doogie_handle_.move(goal_);
  return true;
}

//I could define this callback inside the base class
void RHSolverPlugin::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  current_cell_ = maze_handle_.globalToLocalCell(position_msg);
  doogie_handle_.setPosition(position_msg);
}

};