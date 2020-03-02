// "Copyright [year] <Copyright Owner>"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <string>

#include "doogie_navigation/move_base.hpp"
#include <boost/geometry/algorithms/distance.hpp>

namespace doogie_navigation {

using doogie_msgs::DoogiePosition;
using doogie_msgs::DoogieOrientation;
using doogie_msgs::DoogiePose;


MoveBase::MoveBase(const std::string &robot_namespace)
  : move_to_goal_action_server_(nh_, "move_base_action_server", false)
  , ph_("~")
  , robot_controller_nh_(robot_namespace)
  , pid_{ (robot_namespace + "/pid/linear"), (robot_namespace + "/pid/angular") }
  , angle_to_turn_(0)
  , global_orientation_(doogie_navigation::NORTH) {
  move_to_goal_action_server_.registerGoalCallback(boost::bind(&MoveBase::receiveGoalCallback, this));
  // move_to_goal_action_server_.registerPreemptCallback(boost::bind(&MoveBase::preemptGoalCallback, this));

  // odom_sub_ = robot_controller_nh_.subscribe("odom", 1, &MoveBase::getOdometryDataCallback, this);
  // cmd_vel_pub_ = robot_controller_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  position_pub_ = nh_.advertise<doogie_msgs::DoogiePosition>("doogie_position", 5);
  this->loadParameters();
  current_row_ = row_init_position_;
  current_column_ = column_init_position_;

  move_to_goal_action_server_.start();
}


void MoveBase::receiveGoalCallback() {
  // if (current_robot_state_ == STOPPED) {
  //   goal_ = move_to_goal_action_server_.acceptNewGoal();
  // }

  // target_pose_.position.x = current_pose_.position.x + 0.18;
  // target_pose_.orientation.w = 1.0;
  // ROS_INFO("Goal received");

  goal_ =  move_to_goal_action_server_.acceptNewGoal();
  move_to_goal_result_.status = false;
  ROS_DEBUG_STREAM("goal received");

  angle_to_turn_ = 0;
  switch (goal_->direction) {

  case doogie_msgs::DoogieMoveGoal::FRONT:
    ROS_DEBUG_STREAM("goal is: FRONT");
    robot_state_ = State::MOVING;
    updatePosition();
    break;
  case doogie_msgs::DoogieMoveGoal::BACK:
    ROS_DEBUG_STREAM("goal is: BACK");
    updateOrientation();
    robot_state_ = State::TURNNING;
    updatePosition();
    angle_to_turn_ = doogie::PI;
    break;
  case doogie_msgs::DoogieMoveGoal::LEFT:
    ROS_DEBUG_STREAM("goal is: LEFT");
    updateOrientation();
    robot_state_ = State::TURNNING;
    updatePosition();
    angle_to_turn_ = doogie::PI/2;
    break;
  case doogie_msgs::DoogieMoveGoal::RIGHT:
    ROS_DEBUG_STREAM("goal is: RIGHT");
    updateOrientation();
    robot_state_ = State::TURNNING;
    updatePosition();
    angle_to_turn_ = -1 * doogie::PI/2;
    break;
  default:
    break;
  }
  double actual_angle = diff_drive_controller.getCurrentNormalizedYawOrientation();
  double corrected_angle_to_turn = angle_to_turn_ + actual_angle;
  if ( corrected_angle_to_turn < 0) {
    corrected_angle_to_turn += (2 * doogie::PI);
  }
  angle_to_turn_ = corrected_angle_to_turn;
  
}

// void MoveBase::preemptGoalCallback() {
//   ROS_INFO("Goal id preempted!");
//   move_to_goal_action_server_.setPreempted();
// }

/* void MoveBase::getOdometryDataCallback(const nav_msgs::Odometry::ConstPtr &odometry_data) {
  current_pose_ = odometry_data->pose.pose;
  
} */
bool MoveBase::isHeadingXAxis() {
  // double global_orientation = robot_position_.orientation;
  ROS_DEBUG_STREAM_THROTTLE(1, "is heading: orientation: " << global_orientation_);
  return (global_orientation_ == DoogieOrientation::NORTH || global_orientation_ == DoogieOrientation::SOUTH);
}

// Not being used
void MoveBase::computeAngle(int direction) {
  int goal_angle = 0;

  switch (direction) {
    case doogie_msgs::DoogieMoveGoal::RIGHT :
      angle_to_turn_ = diff_drive_controller.getCurrentNormalizedYawOrientation() - 1.57;
      angle_to_turn_ = angles::normalize_angle_positive(angle_to_turn_);
      is_clockwise_ = true;
      ROS_INFO_STREAM (angle_to_turn_); 
      break;
    case doogie_msgs::DoogieMoveGoal::LEFT:
      angle_to_turn_ = diff_drive_controller.getCurrentNormalizedYawOrientation() + 1.57;
      angle_to_turn_ = angles::normalize_angle_positive(angle_to_turn_);  // angle_to_turn is being normilized two times, why?
      is_clockwise_ = false;
      ROS_INFO_STREAM (angle_to_turn_); 
      break;
  }
  return;
}

bool MoveBase::turnRobot(bool is_clockwise) {
  double control_action_input = 0;
  double actual_angle = diff_drive_controller.getCurrentNormalizedYawOrientation();
  ROS_DEBUG_STREAM_THROTTLE(0.25, "angle to turn: " << angle_to_turn_);
  if(is_clockwise) {
    control_action_input = -0.5;
  } else {
    control_action_input = 0.5;
  }
  double absolute_error = std::abs(angle_to_turn_ - actual_angle);
  diff_drive_controller.setAngularVelocity(control_action_input);
  ROS_DEBUG_STREAM_THROTTLE(0.25, "current angle: " << actual_angle);
  ROS_DEBUG_STREAM_THROTTLE(0.25, "Angular Error: " << absolute_error);
  ROS_DEBUG_STREAM_THROTTLE(0.25, "control_action_input: " << control_action_input);
  if (absolute_error <= angle_tolerance_) {
    ROS_DEBUG_STREAM_THROTTLE(0.25, "Done -> Angular Error: " << absolute_error);
    return false;

  } else {
    ROS_INFO_STREAM_THROTTLE(1, "pub turning");
    return true;
  }
}

void MoveBase::turnRobot(double rad_angle) {
  double current_orientation = diff_drive_controller.getCurrentYawOrientation();
  double control_action_input = pid_[ANGULAR].computeControlAction(rad_angle, current_orientation);
  while( isTurnRobotGoalReached() ) {
    diff_drive_controller.setAngularVelocity(control_action_input);
    diff_drive_controller.publishVelocityMessage();
    current_orientation = diff_drive_controller.getCurrentYPosition();
    control_action_input = pid_[ANGULAR].computeControlAction(current_orientation);
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");
    
    spin();
  }
}

bool MoveBase::isTurnRobotGoalReached() {

  double actual_orientation = getCurrentOrientation();
  ROS_DEBUG_STREAM_THROTTLE(1, "Actual Orientation: " << actual_orientation);

  return (pid_[ANGULAR].computeAbsoluteError(actual_orientation) <= angle_tolerance_);
}

double MoveBase::getCurrentOrientation() {
  return diff_drive_controller.getCurrentYawOrientation();
}

double MoveBase::computeDistanceToMove() {
  int number_of_cells = goal_->cells;
  ROS_DEBUG_STREAM_THROTTLE(1, "number of cells: " << number_of_cells);
  double distance_to_move = 0;

  if (global_orientation_ == GlobalOrientation::WEST) {
    distance_to_move = diff_drive_controller.getCurrentYPosition() + (number_of_cells * cell_size_);
    return distance_to_move;

  } else if (global_orientation_ == GlobalOrientation::EAST) {
    distance_to_move = diff_drive_controller.getCurrentYPosition() - (number_of_cells * cell_size_);
    return distance_to_move;

  } else if (global_orientation_ == GlobalOrientation::SOUTH) {
    distance_to_move = diff_drive_controller.getCurrentXPosition() - (number_of_cells * cell_size_);
    return distance_to_move;

  }
  
  distance_to_move = diff_drive_controller.getCurrentXPosition() + (number_of_cells * cell_size_);
  return distance_to_move;
}

void MoveBase::moveForward() {
  double setpoint = computeDistanceToMove();
  pid_[LINEAR].setSetPoint(setpoint);
  double actual_position = getCurrentPosition();

  while(!isMoveForwardGoalReached()) {
    ROS_DEBUG_STREAM_THROTTLE(1, "orientation: " << global_orientation_);
    double control_action_input = computeLinearControlAction();
    diff_drive_controller.setLinearVelocity(control_action_input);
    diff_drive_controller.publishVelocityMessage();
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");
    
    spin();
  }
  pid_[LINEAR].reset();
  ROS_DEBUG_STREAM_THROTTLE(1, "Done MoveForward: ");
  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Error: " << pid_[LINEAR].getAbsoluteError() );
  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Tolerance: " << distance_tolerance_);
}

void MoveBase::moveHeadingXAxis(double goal_position) {
  double setpoint = goal_position;
  pid_[LINEAR].setSetPoint(setpoint);

  double current_position = 0;

  while(!isMoveForwardGoalReached()) {
    current_position = diff_drive_controller.getCurrentXPosition();
    double control_action_input = pid_[LINEAR].computeControlAction(current_position);

    diff_drive_controller.setLinearVelocity(control_action_input);
    diff_drive_controller.publishVelocityMessage();
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");
    
    spin();
  }

  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Error: " << pid_[LINEAR].getAbsoluteError() );
  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Tolerance: " << distance_tolerance_);
}

void MoveBase::moveHeadingYAxis(double goal_position) {
  double setpoint = goal_position;
  pid_[LINEAR].setSetPoint(setpoint);

  double current_position = current_position = diff_drive_controller.getCurrentYPosition();
  double control_action_input = pid_[LINEAR].computeControlAction(current_position);
  while(!isMoveForwardGoalReached()) {
    diff_drive_controller.setLinearVelocity(control_action_input);
    diff_drive_controller.publishVelocityMessage();
    current_position = diff_drive_controller.getCurrentYPosition();
    control_action_input = pid_[LINEAR].computeControlAction(current_position);
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");
    
    spin();
  }

  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Error: " << pid_[LINEAR].getAbsoluteError() );
  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Tolerance: " << distance_tolerance_);
}

double MoveBase::computeLinearControlAction() {
  double control_action_input = 0;
  double distance_to_move = pid_[LINEAR].getSetPoint(); 
  ROS_DEBUG_STREAM_THROTTLE(0.5, "distance to move: " << distance_to_move);

  control_action_input = pid_[LINEAR].computeControlAction( distance_to_move, getCurrentPosition() );
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Velocity: " << control_action_input);

  if (global_orientation_ == GlobalOrientation::SOUTH || global_orientation_ == GlobalOrientation::EAST) {
    control_action_input *= -1;
  }

  return control_action_input;
}

double MoveBase::getCurrentPosition() {
  double current_position = 0;
  if(isHeadingXAxis()) {
    ROS_DEBUG_STREAM_THROTTLE(0.5, "current x position: " << diff_drive_controller.getCurrentXPosition() );
    current_position = diff_drive_controller.getCurrentXPosition();

  } else {
    ROS_DEBUG_STREAM_THROTTLE(0.5, "current y position: " << diff_drive_controller.getCurrentYPosition() );
    current_position = diff_drive_controller.getCurrentYPosition();
  }
  ROS_DEBUG_STREAM_THROTTLE(0.5, "current position: " << current_position);
  return current_position;
}

bool MoveBase::isMoveForwardGoalReached() {
  double actual_position = getCurrentPosition();
  pid_[LINEAR].computeError(actual_position);
  ROS_DEBUG_STREAM_THROTTLE(0.5, "isMoveForwardGoalReached: ");
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Linear Error: " << pid_[LINEAR].getAbsoluteError() );
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Linear Tolerance: " << distance_tolerance_);
  ROS_DEBUG_STREAM_THROTTLE(0.5, "distance to move: " << pid_[LINEAR].getSetPoint());

  return (pid_[LINEAR].getAbsoluteError() <= distance_tolerance_);
}

void MoveBase::updateOrientation() {
  switch (global_orientation_) {

    case GlobalOrientation::NORTH:
      switch (goal_->direction) {
        case doogie_msgs::DoogieMoveGoal::RIGHT:
          global_orientation_ = GlobalOrientation::EAST;
          angle_to_turn_ = (3 * doogie::PI)/2;
          is_clockwise_ = true;
          return;
          break;
        case doogie_msgs::DoogieMoveGoal::LEFT:
          global_orientation_ = GlobalOrientation::WEST;
          angle_to_turn_ = doogie::PI/2;
          is_clockwise_ = false;
          return;
          break;
        case doogie_msgs::DoogieMoveGoal::BACK:
          global_orientation_ = GlobalOrientation::SOUTH;
          angle_to_turn_ = doogie::PI;
          is_clockwise_ = false;
          return;
          break;
        default:
          break;
      }
      break;

    case GlobalOrientation::SOUTH:
      switch (goal_->direction) {
        case doogie_msgs::DoogieMoveGoal::RIGHT:
          global_orientation_ = GlobalOrientation::WEST;
          angle_to_turn_ = doogie::PI/2;
          is_clockwise_ = true;
          return;
          break;
        case doogie_msgs::DoogieMoveGoal::LEFT:
          global_orientation_ = GlobalOrientation::EAST;
          angle_to_turn_ = (3 * doogie::PI)/2;
          is_clockwise_ = false;
          return;
          break;
        case doogie_msgs::DoogieMoveGoal::BACK:
          global_orientation_ = GlobalOrientation::NORTH;
          angle_to_turn_ = 2 * doogie::PI;
          is_clockwise_ = false;
          return;
          break;
        default:
          break;
      }
      break;
  
  case GlobalOrientation::EAST:
    switch (goal_->direction) {
      case doogie_msgs::DoogieMoveGoal::RIGHT:
        global_orientation_ = GlobalOrientation::SOUTH;
        angle_to_turn_ = doogie::PI;
        is_clockwise_ = true;
        return;
        break;
      case doogie_msgs::DoogieMoveGoal::LEFT:
        global_orientation_ = GlobalOrientation::NORTH;
        angle_to_turn_ = 2 * doogie::PI;
        is_clockwise_ = false;
        return;
        break;
      case doogie_msgs::DoogieMoveGoal::BACK:
        global_orientation_ = GlobalOrientation::WEST;
        angle_to_turn_ = doogie::PI/2;
        is_clockwise_ = false;
        return;
        break;
      default:
        break;
    }
    break;

  case GlobalOrientation::WEST:

    switch (goal_->direction) {
      case doogie_msgs::DoogieMoveGoal::RIGHT:
        global_orientation_ = GlobalOrientation::NORTH;
        angle_to_turn_ = 0.0;
        is_clockwise_ = true;
        return;
        break;
      case doogie_msgs::DoogieMoveGoal::LEFT:
        global_orientation_ = GlobalOrientation::SOUTH;
        angle_to_turn_ = doogie::PI;
        is_clockwise_ = false;
        return;
        break;
      case doogie_msgs::DoogieMoveGoal::BACK:
        global_orientation_ = GlobalOrientation::EAST;
        angle_to_turn_ = (3 * doogie::PI)/2;
        is_clockwise_ = false;
        return;
        break;
      default:
        break;
    }
    break;
  }
}

void MoveBase::updatePosition() {

  switch (global_orientation_) {
    case GlobalOrientation::NORTH:
      current_row_ += goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::NORTH;
      break;
    case GlobalOrientation::SOUTH:
      current_row_ -= goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::SOUTH;
      break;
    case GlobalOrientation::EAST:
      current_column_ += goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::EAST;
      break;
    case GlobalOrientation::WEST:
      current_column_ -= goal_->cells;
      robot_position_.orientation = doogie_msgs::DoogiePosition::WEST;
      break;
    default:
      break;
  }
  
  if ( current_row_ < 1) {
    current_row_ = 0;
  }

  if ( current_column_ < 1) {
    current_column_ = 0;
  }

  robot_position_.row = current_row_;
  robot_position_.column = current_column_;
}

void MoveBase::start() {
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
      if (!turnRobot(is_clockwise_)) {
        diff_drive_controller.stop();
        robot_state_ = State::MOVING;
      }
      break;

    case State::MOVING:
      ROS_INFO_STREAM_THROTTLE(1, "moving");
      moveForward();
      diff_drive_controller.stop();
      position_pub_.publish(robot_position_);
      robot_state_ = State::IDLE;
      break;
  }
    diff_drive_controller.publishVelocityMessage();

    ros::spinOnce();
    rate.sleep();
  }
}

void MoveBase::spin() {
  ros::Rate rate(loop_frequency_);
  ros::spinOnce();
  rate.sleep();
}

void MoveBase::loadParameters() {
  checkParameters(ph_, "distance_tolerance", &distance_tolerance_);
  checkParameters(ph_, "angle_tolerance", &angle_tolerance_);
  checkParameters(ph_, "linear_velocity", &linear_velocity_);
  checkParameters(ph_, "angular_velocity", &angular_velocity_);
  checkParameters(ph_, "loop_frequency", &loop_frequency_);
  checkParameters(ph_, "cell_size", &cell_size_);
  checkParameters(ph_, "row_init_position", &row_init_position_);
  checkParameters(ph_, "column_init_position", &column_init_position_);
  
}

void MoveBase::configureControllers() {
  pid_[LINEAR].setTolerance(distance_tolerance_);
  pid_[ANGULAR].setTolerance(distance_tolerance_);

}

}  // namespace doogie_navigation
