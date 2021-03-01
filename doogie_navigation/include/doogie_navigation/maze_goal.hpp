#ifndef DOOGIE_NAVIGATION_MAZE_GOAL_HPP_
#define DOOGIE_NAVIGATION_MAZE_GOAL_HPP_

#include <ros/ros.h>

#include "doogie_msgs/DoogieMoveGoal.h"
#include "doogie_msgs/DoogieMoveResult.h"
#include "doogie_msgs/DoogieMoveFeedback.h"

#include "doogie_navigation/maze_pose.hpp"


namespace doogie_navigation {
  class MazeGoal {
   public:
    using Direction = doogie::Direction;

    void acceptNewGoal(const doogie_msgs::DoogieMoveGoalConstPtr& goal);
    Direction getDirection() const;
    int8_t getNumberOfCells() const;
    doogie_msgs::DoogieMoveResult getResult() const;
    void finish();
    void operator=(const doogie_msgs::DoogieMoveGoalConstPtr& goal) {
      acceptNewGoal(goal);
    }
    bool empty() {
      return !(goal_);
    }

   private:
    doogie_msgs::DoogieMoveGoalPtr goal_{nullptr};
    doogie_msgs::DoogieMoveResult goal_result_;
    doogie_msgs::DoogieMoveFeedback goal_feedback_;
  }; 
}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MAZE_GOAL_HPP_