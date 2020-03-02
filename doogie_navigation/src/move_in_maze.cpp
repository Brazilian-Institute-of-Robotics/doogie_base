// "Copyright [year] <Copyright Owner>"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <string>

#include "doogie_navigation/move_in_maze.hpp"
#include <boost/geometry/algorithms/distance.hpp>

namespace doogie_navigation {

using doogie_msgs::DoogiePosition;
using doogie_msgs::DoogieOrientation;
using doogie_msgs::DoogiePose;


MoveInMaze::MoveInMaze(const std::string &robot_namespace)
  : MoveBase::MoveBase(robot_namespace)
  , move_base_action_server(nh_)
  , ph_("~") {

  position_pub_ = nh_.advertise<doogie_msgs::DoogiePosition>("doogie_position", 5);
  this->loadParameters();
  robot_pose_.position.row = row_init_position_;
  robot_pose_.position.column = column_init_position_;
  robot_pose_.orientation.direction = DoogieOrientation::NORTH;
  initializeCommandMap();

}

void MoveInMaze::initializeCommandMap() {
command_map[DoogieOrientation::NORTH] =  std::bind(&MoveInMaze::sendMoveToNorthCommand, this, std::placeholders::_1);
command_map[DoogieOrientation::SOUTH] =  std::bind(&MoveInMaze::sendMoveToSouthCommand, this, std::placeholders::_1);
command_map[DoogieOrientation::EAST]  =  std::bind(&MoveInMaze::sendMoveToEastCommand, this, std::placeholders::_1);
command_map[DoogieOrientation::WEST]  =  std::bind(&MoveInMaze::sendMoveToWestCommand, this, std::placeholders::_1);

}

void MoveInMaze::receiveGoalCallback() {

  move_base_action_server.acceptNewGoal();
  ROS_DEBUG_STREAM("goal received");

  switch ( move_base_action_server.getCurrentGoalDirection() ) {

  case doogie_msgs::DoogieMoveGoal::FRONT:
    ROS_DEBUG_STREAM("goal is: FRONT");
    robot_state_ = State::MOVING;
    break;
  case doogie_msgs::DoogieMoveGoal::BACK:
    ROS_DEBUG_STREAM("goal is: BACK");
    updateOrientation();
    robot_state_ = State::TURNNING;
    break;
  case doogie_msgs::DoogieMoveGoal::LEFT:
    ROS_DEBUG_STREAM("goal is: LEFT");
    updateOrientation();
    robot_state_ = State::TURNNING;
    updatePosition();
    break;
  case doogie_msgs::DoogieMoveGoal::RIGHT:
    ROS_DEBUG_STREAM("goal is: RIGHT");
    updateOrientation();
    robot_state_ = State::TURNNING;
    updatePosition();
    break;
  }
  
}

void MoveInMaze::moveStraight() {
  int number_of_cells = move_base_action_server.getCurrentGoalNumberOfCells();
  int distance_to_move = cell_size_ * number_of_cells;
  auto doogie_orientation = getCurrentDoogieOrientation();
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

void MoveInMaze::updatePosition() {
  int current_row = getCurrentDoogiePosition().row;
  int current_column = getCurrentDoogiePosition().column;
  switch (global_orientation_) {
    case GlobalOrientation::NORTH:
      current_row += goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::NORTH;
      break;
    case GlobalOrientation::SOUTH:
      current_row -= goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::SOUTH;
      break;
    case GlobalOrientation::EAST:
      current_column += goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::EAST;
      break;
    case GlobalOrientation::WEST:
      current_column -= goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::WEST;
      break;
  }

  position_pub_.publish(robot_position_);
}

DoogiePosition MoveInMaze::getCurrentDoogiePosition() {
  DoogiePosition current_position = robot_pose_.position;
  ROS_DEBUG_STREAM_THROTTLE(1, "current position: " << current_position);
  return current_position;
}

void MoveInMaze::rotate() {
  auto current_orientation = (getCurrentDoogieOrientation() -1) * doogie::PI/2;
  auto goal_direction = move_base_action_server.getCurrentGoalDirection();

  double goal_angle = current_orientation + goal_direction*(doogie::PI/2);

  turnRobot(goal_angle);
}

const int8_t MoveInMaze::getCurrentDoogieOrientation() const {
  int8_t current_orientation = robot_pose_.orientation.direction;
  return current_orientation;
}

void MoveInMaze::setCurrentDoogieOrientation(int8_t current_orientation) {
  robot_pose_.orientation.direction = current_orientation;
}

void MoveInMaze::updateOrientation() {
  auto goal_direction = move_base_action_server.getCurrentGoalDirection();
  auto doogie_orientation = getCurrentDoogieOrientation();
  auto new_direction = doogie::computeResultDirection(doogie_orientation, goal_direction);

  setCurrentDoogieOrientation(new_direction);
}

void MoveInMaze::start() {
  ROS_DEBUG_STREAM("Start");
  ros::Rate rate(loop_frequency_);

  ROS_INFO("Move base node has started");
  while (ros::ok()) {

  switch (robot_state_) {
    case State::IDLE:
      diff_drive_controller.stop();
      ROS_INFO_STREAM_THROTTLE(1, "idle");
      break;

    case State::TURNNING:
      ROS_INFO_STREAM_THROTTLE(1, "turning");
      rotate();
      diff_drive_controller.stop();
      updateOrientation();
      robot_state_ = State::MOVING;
      
      break;

    case State::MOVING:
      ROS_INFO_STREAM_THROTTLE(1, "moving");
      moveStraight();
      diff_drive_controller.stop();
      updatePosition();
      robot_state_ = State::IDLE;
      break;
  }
    diff_drive_controller.publishVelocityMessage();

    ros::spinOnce();
    rate.sleep();
  }
}

void MoveInMaze::spin() {
  ros::Rate rate(loop_frequency_);
  ros::spinOnce();
  rate.sleep();
}

MoveInMaze::DoogieMoveBaseServer::DoogieMoveBaseServer(ros::NodeHandle nh) 
: move_base_server_controller_(nh, "move_base_action_server", false) {

  move_base_server_controller_.registerGoalCallback(boost::bind(&MoveInMaze::receiveGoalCallback, this));
  move_base_server_controller_.start();
}

MoveInMaze::DoogieMoveBaseServer::~DoogieMoveBaseServer() {

}

void MoveInMaze::DoogieMoveBaseServer::acceptNewGoal() {
  current_goal_ = move_base_server_controller_.acceptNewGoal();
  if(current_goal_->cells == 0) {
    current_goal_->cells = 1;
  }
  current_goal_result_->status = false;
}

const int8_t MoveInMaze::DoogieMoveBaseServer::getCurrentGoalDirection() const {
  return current_goal_->direction;
}

const int8_t MoveInMaze::DoogieMoveBaseServer::getCurrentGoalNumberOfCells() const {
  return current_goal_->cells;
}

const bool MoveInMaze::DoogieMoveBaseServer::getCurrentGoalStatus() const {
  return current_goal_result_->status;
}

void MoveInMaze::DoogieMoveBaseServer::setSucceeded() {
  current_goal_result_->status = true;
}

}  // namespace doogie_navigation
