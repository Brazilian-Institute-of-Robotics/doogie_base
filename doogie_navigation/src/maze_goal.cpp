#include "doogie_navigation/maze_goal.hpp"
namespace doogie_navigation {
using Direction = doogie::Direction;

void MazeGoal::acceptNewGoal(const doogie_msgs::DoogieMoveGoalConstPtr& goal) {
  goal_ = boost::make_shared<doogie_msgs::DoogieMoveGoal>(*goal);
  if(goal_->cells == 0) {
    goal_->cells = 1;
  }
  goal_result_.status = false;
}

Direction MazeGoal::getDirection() const {
  return (Direction)goal_->direction;
}

int8_t MazeGoal::getNumberOfCells() const {
  return goal_->cells;
}

doogie_msgs::DoogieMoveResult MazeGoal::getResult() const {
  return goal_result_;
}

void MazeGoal::finish() {
  goal_result_.status = true;
  goal_.reset();
}
}// namespace doogie_navigation
