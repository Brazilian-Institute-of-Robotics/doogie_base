#include "doogie_core/base_solver.hpp"

namespace doogie_core{


BaseSolver::BaseSolver(const MazeMatrixPtr maze_matrix)
: matrix_handle_(maze_matrix){
  
}

BaseSolver::BaseSolver(){
  doogie_position_sub_ = nh_.subscribe("doogie_position", 100, &BaseSolver::doogiePositionCallback, this);
  maze_matrix_sub_ = nh_.subscribe("maze_matrix",2, &BaseSolver::mazeMatrixCallback, this);
}

int& BaseSolver::ROSParams::getPlanAttempts(){
  return plan_attempts;
}

float& BaseSolver::ROSParams::getRate(){
  return rate;
}

void BaseSolver::initialize(){
  configureSolverFromParams();
}

void BaseSolver::configureSolverFromParams(){
  if(!ph_.getParam("plan_attempts", params_.getPlanAttempts())){
    ROS_INFO_STREAM("/plan_attempts param is not set. Using default: " << params_.plan_attempts);
  }

  if(!ph_.getParam("plan_attempts", params_.getRate())){
    ROS_INFO_STREAM("/rate param is not set. Using default: " << params_.plan_attempts);
  }
}

void BaseSolver::waitForStart(){
  ROS_INFO("Waiting for doogie_position...");
  while(!start_solver){
    sleep();
  }
}

bool BaseSolver::isWallFront(){
  return current_cell_.front_wall;
}

bool BaseSolver::isWallBack(){
  return current_cell_.back_wall;
}

bool BaseSolver::isWallLeft(){
  return current_cell_.left_wall;  
}

bool BaseSolver::isWallRight(){
  return current_cell_.right_wall;  
}

void BaseSolver::sleep(){
  ros::Rate(params_.getRate()).sleep();
  ros::spinOnce();
}

void BaseSolver::setSolverName(const std::string& solver_name){
  solver_name_ = solver_name;
}

std::string BaseSolver::getSolverName(){
  return solver_name_;
}

// Would be better retrieve doogie_position from a service instead of topic publication
void BaseSolver::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  current_cell_ = matrix_handle_.getLocalCell(position_msg);
  doogie_handle_.setPosition(position_msg);
}

void BaseSolver::mazeMatrixCallback(const doogie_msgs::MazeCellMultiArray& matrix_maze){
  matrix_handle_.updateMatrix(matrix_maze);
}

} //namespace doogie_core