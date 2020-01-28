#include <pluginlib/class_list_macros.h>
#include "doogie_algorithms/right_hand_solver_plugin/right_hand_solver_plugin.hpp"

using namespace doogie_algorithms;

namespace doogie_algorithms{

// register RightHandSolver as a BaseSolver implementation
PLUGINLIB_EXPORT_CLASS(right_hand_solver_plugin::RightHandSolverPlugin, doogie_core::BaseSolver)

namespace right_hand_solver_plugin{

RightHandSolverPlugin::RightHandSolverPlugin(){}

//to do: turn it void()
bool RightHandSolverPlugin::makePlan(){
  int plan_attempts = 0;
  while(!isPlanAttemptsReached(plan_attempts)){
    if (!isWallRight()){
      goal_.direction = doogie_msgs::DoogieMoveGoal::RIGHT;
      ROS_DEBUG_STREAM("Goal is: right");

      return true;
    }

    else if (!isWallFront()){
      goal_.direction = doogie_msgs::DoogieMoveGoal::FRONT;
      ROS_DEBUG_STREAM("Goal is: front");
      return true;
    }

    else if (!isWallLeft()){
      goal_.direction = doogie_msgs::DoogieMoveGoal::LEFT;
      ROS_DEBUG_STREAM("Goal is: left");
      return true;
    }

    else{
      plan_attempts++;
      ros::spinOnce();
    }
  }

    return false;
  
}

bool RightHandSolverPlugin::isPlanAttemptsReached(int count){
  if(count >= params_.plan_attempts){
    ROS_FATAL("Could not make a plan. There is no path robot can move");
    // throw std::runtime_error("Could not make a plan. There is no path robot can move");
  }
  return count >= params_.plan_attempts;
}

//to do: turn it void
bool RightHandSolverPlugin::move(){
  
  goal_.cells=1; //temporary: move_base_node shall have number of cells as 1 by default.
  doogie_handle_.move(goal_);

  return true;
}

void RightHandSolverPlugin::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  ROS_DEBUG("update doogie");
  start_solver = true; //to do: avoid using flags
  doogie_handle_.setPosition(position_msg);
}

void RightHandSolverPlugin::mazeMatrixCallback(const doogie_msgs::MazeCellMultiArray& matrix_maze){
  ROS_DEBUG("update matrix");
  matrix_handle_.updateMatrix(matrix_maze);
  current_cell_ = matrix_handle_.getLocalCell(doogie_handle_.getPosition());
}

} // namespace doogie_algorithms::right_hand_solver_plugin
} // namespace doogie_algorithms