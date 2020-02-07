#include "doogie_core/base_solver.hpp"

namespace doogie_core{

BaseSolver::BaseSolver() : rate_(50){
  doogie_position_sub_ = nh_.subscribe("doogie_position", 100, &BaseSolver::doogiePositionCallback, this);
}

bool BaseSolver::getParams(){
  
}

bool BaseSolver::isWallFront(){
  if(current_cell_.front_wall){
    return true;
  }
  return false;
}

bool BaseSolver::isWallLeft(){
  if(current_cell_.left_wall){
    return true;
  }
  return false;  
}

bool BaseSolver::isWallRight(){
  if(current_cell_.right_wall){
    return true;
  }
  return false;  
}

void BaseSolver::sleep(){
  rate_.sleep();
}

void BaseSolver::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  // current_cell_ = matrix_handle_.globalToLocal(position_msg);
  doogie_handle_.setPosition(position_msg);
}

}; 