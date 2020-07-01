// "Copyright [year] <Copyright Owner>"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <string>

#include "doogie_navigation/move_in_maze.hpp"
#include <boost/geometry/algorithms/distance.hpp>

namespace doogie_navigation {
using Pose = doogie::doogie_navigation::MazePose;
using GlobalOrientation = doogie::GlobalOrientation;
using Direction = doogie::Direction;

MoveInMaze::MoveInMaze(const std::string &robot_namespace, const ros::NodeHandle& robot_nh)
  : MoveBase::MoveBase(robot_namespace, robot_nh)
  , move_base_action_server(nh_, "move_base_action_server", false) {
  
  move_base_action_server.registerGoalCallback(boost::bind(&MoveInMaze::receiveGoalCallback, this));
  move_base_action_server.start();

  position_pub_ = nh_.advertise<doogie_msgs::DoogiePosition>("doogie_position", 5);
  robot_pose_ = Pose{params.row_init_position, params.row_init_position, GlobalOrientation::NORTH};
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
  updateOrientation(goal_.getDirection());
  updatePosition(goal_.getNumberOfCells());
  robot_state_ = State::TURNNING;
}

void MoveInMaze::moveStraight() {
  auto number_of_cells = goal_.getNumberOfCells();
  auto distance_to_move = params.cell_size * number_of_cells;
  auto doogie_orientation = robot_pose_.getGlobalOrientation();
  command_map[doogie_orientation](distance_to_move);
}

void MoveInMaze::sendMoveToNorthCommand(const double& distance_to_move) {
  double goal_position = diff_drive_controller.getCurrentXPosition() + distance_to_move;
  moveHeadingXAxis(goal_position);
}

void MoveInMaze::sendMoveToSouthCommand(const double& distance_to_move) {
  double goal_position = diff_drive_controller.getCurrentXPosition() - distance_to_move;
  moveHeadingXAxis(goal_position);
}

void MoveInMaze::sendMoveToEastCommand(const double& distance_to_move) {
  double goal_position = diff_drive_controller.getCurrentYPosition() - distance_to_move;
  moveHeadingYAxis(goal_position);
}

void MoveInMaze::sendMoveToWestCommand(const double& distance_to_move) {
  double goal_position = diff_drive_controller.getCurrentYPosition() + distance_to_move;
  moveHeadingYAxis(goal_position);
}

void MoveInMaze::updatePosition(int cells) {
  switch (global_orientation_) {
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

  position_pub_.publish(robot_position_);
}

void MoveInMaze::rotate(Direction goal_direction) {
  auto current_orientation = (robot_pose_.getGlobalOrientation() -1) * doogie::PI/2;
  double goal_angle = current_orientation + goal_direction*(doogie::PI/2);

  turnRobot(goal_angle);
}

void MoveInMaze::updateOrientation(Direction goal_direction) {
  robot_pose_.transformOrientation(goal_direction);
}

void MoveInMaze::start() {
  ROS_DEBUG_STREAM("Start");
  static ros::Rate rate{params.loop_frequency};

  ROS_INFO("Move base node has started");
  while (ros::ok()) {
    if (!goal_.empty()) {

      if (robot_state_ == State::IDLE) {
        diff_drive_controller.stop();
        return;
      }

      if (robot_state_ == State::TURNNING) {
        rotate(goal_.getDirection());
        diff_drive_controller.stop();
        updateOrientation(goal_.getDirection());
        return;
      }

      if (robot_state_ == State::MOVING) {
        moveStraight();
        diff_drive_controller.stop();
        updatePosition(goal_.getNumberOfCells());
        robot_state_ = State::IDLE;
        return;
      }
    }
    diff_drive_controller.publishVelocityMessage();

    ros::spinOnce();
    rate.sleep();
  }
}

void MoveInMaze::spin() {
  ros::Rate rate(params.loop_frequency);
  ros::spinOnce();
  rate.sleep();
}

}  // namespace doogie_navigation
