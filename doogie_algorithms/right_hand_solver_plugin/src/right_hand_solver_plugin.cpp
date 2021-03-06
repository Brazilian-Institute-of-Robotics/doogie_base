#include <pluginlib/class_list_macros.h>

#include "doogie_algorithms/right_hand_solver_plugin/right_hand_solver_plugin.hpp"

using namespace doogie_algorithms;

namespace doogie_algorithms{

// register RightHandSolver as a BaseSolver implementation
PLUGINLIB_EXPORT_CLASS(right_hand_solver_plugin::RightHandSolverPlugin, doogie_algorithms::BaseSolver)

namespace right_hand_solver_plugin {

RightHandSolverPlugin::RightHandSolverPlugin() {}

bool RightHandSolverPlugin::makePlan() {
  int plan_attempts = 0;
  while (!isPlanAttemptsReached(plan_attempts)) {
    if (!isWallRight()) {
      goal_.direction = doogie_msgs::DoogieMoveGoal::RIGHT;
      ROS_INFO_STREAM("Goal is: right");

      return true;
    }

    if (!isWallFront()) {
      goal_.direction = doogie_msgs::DoogieMoveGoal::FRONT;
      ROS_INFO_STREAM("Goal is: front");
      return true;
    }

    if (!isWallLeft()) {
      goal_.direction = doogie_msgs::DoogieMoveGoal::LEFT;
      ROS_INFO_STREAM("Goal is: left");
      return true;
    }

    if (!isWallBack()) {
      goal_.direction = doogie_msgs::DoogieMoveGoal::BACK;
      ROS_INFO_STREAM("Goal is: back");
      return true;
    }

    plan_attempts++;
    ros::spinOnce();
  }

    return false;
}

bool RightHandSolverPlugin::isPlanAttemptsReached(int count) {
  if (count >= params_.plan_attempts) {
    ROS_FATAL("Could not make a plan. There is no path robot can move");
    throw std::runtime_error("");
  }
  return count >= params_.plan_attempts;
}

// to do: turn it void
bool RightHandSolverPlugin::move() {
  goal_.cells = 1;
  getDoogieHandle().move(goal_);

  return true;
}

}  // namespace right_hand_solver_plugin
}  // namespace doogie_algorithms
