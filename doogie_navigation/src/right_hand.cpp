#include "doogie_navigation/right_hand.hpp"

namespace doogie_navigation{
void RightHandStrategy::init(){
  maze_walls_sub_ = nh_.subscribe("maze_walls_matrix",100, &RightHandStrategy::mazeWallsCallback, this);
  doogie_position_sub_ = nh_.subscribe("doogie_position", 100, &RightHandStrategy::doogiePositionCallback, this);

}

void RightHandStrategy::mazeWallsCallback(const doogie_msgs::MazeCellMultiArray& matrix_msg){
  *matrix_maze_ = matrix_msg;
}

void RightHandStrategy::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  current_cell_ = matrix_handle_.GlobalToLocal(position_msg);
  doogie_handle_.setPosition(position_msg);
}

}