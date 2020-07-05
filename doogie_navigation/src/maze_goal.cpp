#include "doogie_navigation/maze_goal.hpp"
namespace doogie_navigation {
using Direction = doogie::Direction;

void MazeGoal::acceptNewGoal(const doogie_msgs::DoogieMoveGoalConstPtr& goal) {
  *goal_ = *goal;
  if(goal_->cells == 0) {
    goal_->cells = 1;
  }
  goal_result_->status = false;
}

Direction MazeGoal::getDirection() const {
  return (Direction)goal_->direction;
}

const int8_t MazeGoal::getNumberOfCells() const {
  return goal_->cells;
}

const bool MazeGoal::getStatus() const {
  return goal_result_->status;
}

void MazeGoal::finish() {
  goal_result_->status = true;
  goal_.reset();
}
}// namespace doogie_navigation
