// "Copyright [year] <Copyright Owner>"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "doogie_msgs/DoogiePose.h"

#include <cmath>
#include <string>

#include "doogie_navigation/move_in_maze.hpp"
#include <boost/geometry/algorithms/distance.hpp>

namespace doogie_navigation {
using Pose = doogie_navigation::MazePose;
using GlobalOrientation = doogie::GlobalOrientation;
using Direction = doogie::Direction;

MoveInMaze::MoveInMaze(const ros::NodeHandle& robot_nh)
: MoveBase::MoveBase(robot_nh)
, move_base_action_server(nh_, "move_base_action_server", false) {
  move_base_action_server.registerGoalCallback(boost::bind(&MoveInMaze::receiveGoalCallback, this));
  move_base_action_server.start();

  pose_pub_ = nh_.advertise<doogie_msgs::DoogiePose>("doogie_pose", 5);
  robot_pose_ = Pose{params_.row_init_position, params_.row_init_position, GlobalOrientation::NORTH};
  initializeCommandMap();
}

void MoveInMaze::initializeCommandMap() {
  command_map[GlobalOrientation::NORTH] =  std::bind(&MoveInMaze::sendMoveToNorthCommand, this, std::placeholders::_1);
  command_map[GlobalOrientation::SOUTH] =  std::bind(&MoveInMaze::sendMoveToSouthCommand, this, std::placeholders::_1);
  command_map[GlobalOrientation::EAST]  =  std::bind(&MoveInMaze::sendMoveToEastCommand, this, std::placeholders::_1);
  command_map[GlobalOrientation::WEST]  =  std::bind(&MoveInMaze::sendMoveToWestCommand, this, std::placeholders::_1);
}

void MoveInMaze::receiveGoalCallback() {
  goal_ = move_base_action_server.acceptNewGoal();
  auto goal_direction = goal_.getDirection();
  ROS_DEBUG_STREAM("goal received");

  if (goal_direction == Direction::FRONT) {
    robot_state_ = State::MOVING;
    return;
  }

  robot_state_ = State::TURNNING;
}

void MoveInMaze::moveStraight(int8_t number_of_cells) {
  auto distance_to_move = params_.cell_size * number_of_cells;
  auto doogie_orientation = robot_pose_.getGlobalOrientation();
  command_map[doogie_orientation](distance_to_move);
}

void MoveInMaze::sendMoveToNorthCommand(double distance_to_move) {
  double goal_position = localization_iface_->getCurrentXPosition() + distance_to_move;
  moveHeadingXAxis(goal_position, Heading::POSITIVE);
}

void MoveInMaze::sendMoveToSouthCommand(double distance_to_move) {
  double goal_position = localization_iface_->getCurrentXPosition() - distance_to_move;
  moveHeadingXAxis(goal_position, Heading::NEGATIVE);
}

void MoveInMaze::sendMoveToEastCommand(double distance_to_move) {
  double goal_position = localization_iface_->getCurrentYPosition() - distance_to_move;
  moveHeadingYAxis(goal_position, Heading::NEGATIVE);
}

void MoveInMaze::sendMoveToWestCommand(double distance_to_move) {
  double goal_position = localization_iface_->getCurrentYPosition() + distance_to_move;
  moveHeadingYAxis(goal_position, Heading::POSITIVE);
}

void MoveInMaze::updatePosition(int cells) {
  auto global_orientation = robot_pose_.getGlobalOrientation();
  switch (global_orientation) {
    case GlobalOrientation::NORTH:
      robot_pose_.incrementRow(cells);
      robot_pose_.setGlobalOrientation(GlobalOrientation::NORTH);
      break;
    case GlobalOrientation::SOUTH:
      robot_pose_.decrementRow(cells);
      robot_pose_.setGlobalOrientation(GlobalOrientation::SOUTH);
      break;
    case GlobalOrientation::EAST:
      robot_pose_.incrementColumn(cells);
      robot_pose_.setGlobalOrientation(GlobalOrientation::EAST);
      break;
    case GlobalOrientation::WEST:
      robot_pose_.decrementColumn(cells);
      robot_pose_.setGlobalOrientation(GlobalOrientation::WEST);
      break;
  }
  auto doogie_pose = robot_pose_.toDoogieMsg();
  ROS_INFO_STREAM("NAVIGATION: Updating Doogie Position to " << doogie_pose.position);
  pose_pub_.publish(doogie_pose);
}

void MoveInMaze::rotate(Direction goal_direction) {
  auto current_orientation = (robot_pose_.getGlobalOrientation() -1) * doogie::PI/2;
  double goal_angle = current_orientation + goal_direction*(doogie::PI/2);
  ROS_INFO_STREAM("NAVIGATION: Current Orientation " << current_orientation);
  ROS_INFO_STREAM("NAVIGATION: Goal Direction " << goal_direction);
  ROS_INFO_STREAM("NAVIGATION: Rotating " << goal_angle);

  turnRobot(-goal_angle);
}

void MoveInMaze::updateOrientation(Direction goal_direction) {
  robot_pose_.transformOrientation(goal_direction);
  auto doogie_pose = robot_pose_.toDoogieMsg();
  ROS_INFO_STREAM("NAVIGATION: Update Orientation to " << (int)doogie_pose.orientation.direction);
  pose_pub_.publish(doogie_pose);
}

void MoveInMaze::start() {
  ROS_DEBUG_STREAM("Start");
  ros::Rate rate{params_.loop_frequency};

  ROS_INFO("Move base node has started");
  while (ros::ok()) {
    if (!goal_.empty()) {

      if (robot_state_ == State::IDLE) {
        ROS_INFO_STREAM("IDLE");
        motion_iface_.stop();
        continue;
      }

      if (robot_state_ == State::TURNNING) {
        ROS_INFO_STREAM("Goal is Rotate");
        rotate(goal_.getDirection());
        motion_iface_.stop();
        updateOrientation(goal_.getDirection());
        robot_state_ = State::MOVING;
        continue;
      }

      if (robot_state_ == State::MOVING) {
        ROS_INFO_STREAM("Goal is Move Straight");
        moveStraight(goal_.getNumberOfCells());
        motion_iface_.stop();
        updatePosition(goal_.getNumberOfCells());
        goal_.finish();
        move_base_action_server.setSucceeded(goal_.getResult());
        robot_state_ = State::IDLE;
        continue;
      }
    }
    motion_iface_.publishVelocityMessage();

    ros::spinOnce();
    rate.sleep();
  }
}

void MoveInMaze::spin() {
  ros::Rate rate(params_.loop_frequency);
  ros::spinOnce();
  rate.sleep();
}

}  // namespace doogie_navigation
