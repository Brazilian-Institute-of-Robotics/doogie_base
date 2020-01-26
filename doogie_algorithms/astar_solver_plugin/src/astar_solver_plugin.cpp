#include "doogie_algorithms/astar_solver_plugin/astar_solver_plugin.hpp"

namespace doogie_algorithms{

namespace astar_solver_plugin{

int CostFromStart::calculateCost(doogie_msgs::MazeCellPtr current_cell){
  return (start_cell_.row - current_cell->row) + (start_cell_.column - current_cell->column);
}

int CostToGoal::calculateCost(doogie_msgs::MazeCellPtr current_cell){
  return (current_cell->row - goal_cell_.row) + (current_cell->column - goal_cell_.column);
}

AStarSolverPlugin::AStarSolverPlugin(){

  open_list_ = doogie_core::MazeMatrixHandle::makeMatrix(20, 20);
  closed_list_ = doogie_core::MazeMatrixHandle::makeMatrix(20, 20);
}

void AStarSolverPlugin::updateOpenList(){
  
  calculateMazeCellCost(current_cell_.toGlobalCell());

  if(!isWallFront){
      open_list_->data.push_back(*current_cell_.toGlobalCell());
  }
  if(!isWallRight){
      open_list_->data.push_back(*current_cell_.toGlobalCell());
  }
  if(!isWallLeft){
      open_list_->data.push_back(*current_cell_.toGlobalCell());
  }
}

bool AStarSolverPlugin::makePlan(){
  
}

void AStarSolverPlugin::calculateMazeCellCost(doogie_msgs::MazeCellPtr maze_cell){
  g.value = CostFromStart::calculateCost(maze_cell);
  h.value = CostToGoal::calculateCost(maze_cell);

  maze_cell->weight= g.value + h.value;
}

void AStarSolverPlugin::doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg){
  doogie_handle_.setPosition(position_msg);
  current_cell_ = matrix_handle_.getLocalCell(position_msg);
}

} // namespace doogie_algorithms::star_solver_plugin
} // namespace doogie_algorithms