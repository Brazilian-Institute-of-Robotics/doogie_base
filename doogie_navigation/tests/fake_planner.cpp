#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>

#include "doogie_navigation/move_base.hpp"
#include "doogie_msgs/DoogieMoveAction.h"

doogie_msgs::DoogieMoveGoal makeFowardFakePlan() {
  doogie_msgs::DoogieMoveGoal fake_goal;
  fake_goal.direction = doogie_msgs::DoogieMoveGoal::RIGHT;
  fake_goal.cells = 1;
  return fake_goal;
}

doogie_msgs::DoogieMoveGoal makeTurnFakePlan() {
  doogie_msgs::DoogieMoveGoal fake_goal;
  fake_goal.direction = doogie_msgs::DoogieMoveGoal::RIGHT;
  return fake_goal;
}

void sendGoalAndWaitUntilSucceed(doogie_msgs::DoogieMoveGoal fake_goal) {
  actionlib::SimpleActionClient<doogie_msgs::DoogieMoveAction> move_base_client_("move_base_action_server");
  ROS_INFO("Waiting for server");
  move_base_client_.waitForServer();
  move_base_client_.sendGoal(fake_goal);
  while(!move_base_client_.getState().isDone() && ros::ok()) {
    ROS_INFO("INFO: FAKE PLANNER");
    ROS_DEBUG_STREAM_THROTTLE(40, "Goal is running.\n State is " + move_base_client_.getState().toString());
    ros::Rate(10).sleep();
  }
  ROS_INFO_STREAM("Goal accomplished");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_planner");
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {

    ros::console::notifyLoggerLevelsChanged();
  }

  /* ROS_DEBUG_STREAM("DEBUG: sending Foward Goal");
  doogie_msgs::DoogieMoveGoal fake_foward_goal = makeFowardFakePlan();
  sendGoalAndWaitUntilSucceed(fake_foward_goal); */

  ROS_DEBUG_STREAM("DEBUG: sending Turn Goal");
  doogie_msgs::DoogieMoveGoal fake_turn_goal = makeTurnFakePlan();
  sendGoalAndWaitUntilSucceed(fake_turn_goal);

}
