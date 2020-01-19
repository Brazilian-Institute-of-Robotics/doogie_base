#include "doogie_algorithms/base_solver.hpp"

namespace doogie_algorithms{

BaseSolver::BaseSolver() : rate_(50){
  doogie_position_sub_ = nh_.subscribe("doogie_position", 100, &BaseSolver::doogiePositionCallback, this);
}

bool BaseSolver::getParams(){
  
}

void BaseSolver::sleep(){
  rate_.sleep();
}

void BaseSolver::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  current_cell_ = matrix_handle_.GlobalToLocal(position_msg);
  doogie_handle_.setPosition(position_msg);
}

};