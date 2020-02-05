#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <string>

#include "doogie_navigation/move_base.hpp"
#include <boost/geometry/algorithms/distance.hpp>

namespace doogie_navigation {

MoveBase::MoveBase(const ros::NodeHandle &robot_controller_nh)
  : move_to_goal_action_server_(nh_, "move_base_action_server", false)
  , ph_("~")
  , robot_controller_nh_(robot_controller_nh)
  , is_to_change_robot_state_(false)
  , angle_to_turn_(0)
  , distance_to_move_(0)
  , moving_foward_(false)
  , turning_(false)
  , global_orientation_(doogie_navigation::NORTH) {
  move_to_goal_action_server_.registerGoalCallback(boost::bind(&MoveBase::receiveGoalCallback, this));
  // move_to_goal_action_server_.registerPreemptCallback(boost::bind(&MoveBase::preemptGoalCallback, this));

  odom_sub_ = robot_controller_nh_.subscribe("odom", 1, &MoveBase::getOdometryDataCallback, this);
  cmd_vel_pub_ = robot_controller_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  this->loadParameters();
  move_to_goal_action_server_.start();
}

void MoveBase::receiveGoalCallback() {
  // if (current_robot_state_ == STOPPED) {
  //   goal_ = move_to_goal_action_server_.acceptNewGoal();
  //   is_to_change_robot_state_ = true;
  // }

  // target_pose_.position.x = current_pose_.position.x + 0.18;
  // target_pose_.orientation.w = 1.0;
  // ROS_INFO("Goal received");

  goal_ = move_to_goal_action_server_.acceptNewGoal();
  is_to_change_robot_state_ = true;
  distance_to_move_ = 0;
  angle_to_turn_ = 0;
  switch (goal_->direction)
  {
  case doogie_msgs::DoogieMoveGoal::FRONT:
    computeDistance(goal_->cells);
    robot_state_ = State::MOVING;
    break;
  case doogie_msgs::DoogieMoveGoal::BACK:
    break;
  case doogie_msgs::DoogieMoveGoal::LEFT:
    // computeAngle(goal_->direction);
    updateOrientation();
    robot_state_ = State::TURNNING;
    break;
  case doogie_msgs::DoogieMoveGoal::RIGHT:
    // computeAngle(goal_->direction);
    updateOrientation();
    robot_state_ = State::TURNNING;
    break;
  default:
    break;
  }

  
}

// void MoveBase::preemptGoalCallback() {
//   ROS_INFO("Goal id preempted!");
//   move_to_goal_action_server_.setPreempted();
// }

// void MoveBase::executeCB(const doogie_msgs::DoogieMoveActionGoalConstPtr &goal) {
//   int direction = goal->goal.direction;

//   if (direction = 3) {
//     angle_to_turn_ = -1.57;
//   } else if (direction = 4) {
//     angle_to_turn_ = 1.57;
//   }

//   ROS_INFO_STREAM("angle_to_turn_");  
//   ROS_INFO_STREAM(direction);    
// } 

void MoveBase::getOdometryDataCallback(const nav_msgs::Odometry::ConstPtr &odometry_data) {
  current_pose_ = odometry_data->pose.pose;
  double current_angle = tf::getYaw(current_pose_.orientation);
  current_angle = angles::normalize_angle_positive(current_angle);
  getOrientation(current_angle);

  switch (robot_state_)
  {
  case State::IDLE :
    twist_cmd_.angular.z = 0;
    twist_cmd_.linear.x = 0;
    cmd_vel_pub_.publish(twist_cmd_);
    ROS_INFO_STREAM_THROTTLE(1, "idle");
    break;
  
  case State::TURNNING:
    ROS_INFO_STREAM_THROTTLE(1, "turning");
    if (!turnRobot(angle_to_turn_, current_angle, is_clockwise_)) {
      robot_state_ = State::MOVING;
      computeDistance(goal_->cells);
    };
    break;

  case State::MOVING:
    ROS_INFO_STREAM_THROTTLE(1, "moving");
    if(!moveForward()) {
      robot_state_ = State::IDLE;
    }
    break;
  
  default:
    break;
  }

  // ROS_INFO_STREAM_THROTTLE(1, "position");
  // ROS_INFO_STREAM_THROTTLE(1, current_position);

  // ROS_INFO_STREAM_THROTTLE(1, "global_orientation");
  // ROS_INFO_STREAM_THROTTLE(1, global_orientation_);
  
}
void MoveBase::getOrientation(double current_angle) {
  
  // between 45º and 135º or 225º and 315º
  if ( ((0.79) <= current_angle && 
        (2.36) >= current_angle) || 
       ((3.93) <= current_angle &&
        (5.5) >= current_angle) )  {
    is_heading_x_ = false;
    return;
  }

  is_heading_x_ = true;
  return;
}

void MoveBase::computeDistance(int cell_number) {
  distance_to_move_ = 0;
  // if(!is_heading_x_) {
    if (global_orientation_ == GlobalOrientation::WEST) {
      distance_to_move_ = current_pose_.position.y + (cell_number * cell_size_);
      return;
    } else if (global_orientation_ == GlobalOrientation::EAST) {
      distance_to_move_ = current_pose_.position.y - (cell_number * cell_size_);
      return;
    }
  // }
  distance_to_move_ = current_pose_.position.x + (cell_number * cell_size_);
  return;
}

void MoveBase::computeAngle(int direction) {
  angle_to_turn_ = 0;
  switch (direction)
  {
  case doogie_msgs::DoogieMoveGoal::RIGHT :
    angle_to_turn_ = angles::normalize_angle_positive(tf::getYaw(current_pose_.orientation)) - 1.57;
    angle_to_turn_ = angles::normalize_angle_positive(angle_to_turn_);
    is_clockwise_ = true;
    ROS_INFO_STREAM (angle_to_turn_); 
    break;
  case doogie_msgs::DoogieMoveGoal::LEFT:
    angle_to_turn_ = angles::normalize_angle_positive(tf::getYaw(current_pose_.orientation)) + 1.57;
    angle_to_turn_ = angles::normalize_angle_positive(angle_to_turn_);
    is_clockwise_= false;
    ROS_INFO_STREAM (angle_to_turn_); 
    break;
  }
  return;
}

bool MoveBase::turnRobot(double target_angle, double current_angle, bool is_clockwise) {
  double error = std::abs(target_angle - current_angle);

  if(is_clockwise) {
    twist_cmd_.angular.z = -angular_gain_ * error;
  }else {
    twist_cmd_.angular.z = angular_gain_ * error;
  }

  // ROS_INFO_STREAM_THROTTLE(1, "current angle");
  // ROS_INFO_STREAM_THROTTLE(1, current_angle);

  // ROS_INFO_STREAM_THROTTLE(1, "angle to turn");
  // ROS_INFO_STREAM_THROTTLE(1, angle_to_turn_);

  // ROS_INFO_STREAM_THROTTLE(1, "error angular");
  // ROS_INFO_STREAM_THROTTLE(1, error);

  if (error >= angle_tolerance_) {
    ROS_INFO_STREAM_THROTTLE(1, "pub turning");
    cmd_vel_pub_.publish(twist_cmd_);
    return true;

  }else {
    twist_cmd_.angular.z = 0;
    cmd_vel_pub_.publish(twist_cmd_);
    return false;
  }
}

bool MoveBase::moveForward() {
  
  // ROS_INFO_STREAM_THROTTLE(1, "is_heading_x_"); 
  // ROS_INFO_STREAM_THROTTLE(1, is_heading_x_); 

  // ROS_INFO_STREAM_THROTTLE(1, "distance_to_move_");
  // ROS_INFO_STREAM_THROTTLE(1, distance_to_move_);
  


  double position_error;
  double current_position; 
  if(!is_heading_x_) {

    position_error = std::abs(distance_to_move_ - current_pose_.position.y);
  } else {
    
    position_error = std::abs(distance_to_move_ - current_pose_.position.x);
  }

  // ROS_INFO_STREAM_THROTTLE(1, "error position");
  // ROS_INFO_STREAM_THROTTLE(1, position_error);

  twist_cmd_.linear.x = linear_gain_ * position_error;
  // ROS_INFO_STREAM(is_to_change_robot_state_);
  // ROS_INFO_STREAM(moving_foward_);
  
  if (position_error >= distance_tolerance_) {
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");
    cmd_vel_pub_.publish(twist_cmd_);
    return true;

  }else {
    twist_cmd_.linear.x = 0;
    cmd_vel_pub_.publish(twist_cmd_);
    return false;
  }
}

void MoveBase::updateOrientation() {
  switch (global_orientation_)
  {
  case GlobalOrientation::NORTH:
  
    switch (goal_->direction)
    {
    case doogie_msgs::DoogieMoveGoal::RIGHT:
      global_orientation_ = GlobalOrientation::EAST;
      angle_to_turn_ = 4.71;
      is_clockwise_ = true;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::LEFT:
      global_orientation_ = GlobalOrientation::WEST;
      angle_to_turn_ = 1.57;
      is_clockwise_ = false;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::BACK:
      global_orientation_ = GlobalOrientation::SOUTH;
      angle_to_turn_ = 3.14;
      is_clockwise_ = false;
      return;
      break;
    default:
      break;
    }
    break;

  case GlobalOrientation::SOUTH:
    
    switch (goal_->direction)
    {
    case doogie_msgs::DoogieMoveGoal::RIGHT:
      global_orientation_ = GlobalOrientation::WEST;
      angle_to_turn_ = 1.57;
      is_clockwise_ = true;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::LEFT:
      global_orientation_ = GlobalOrientation::EAST;
      angle_to_turn_ = 4.71;
      is_clockwise_ = false;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::BACK:
      global_orientation_ = GlobalOrientation::NORTH;
      angle_to_turn_ = 0.0;
      is_clockwise_ = false;
      return;
      break;
    default:
      break;
    }
    break;
  
  case GlobalOrientation::EAST:
    
    switch (goal_->direction)
    {
    case doogie_msgs::DoogieMoveGoal::RIGHT:
      global_orientation_ = GlobalOrientation::SOUTH;
      angle_to_turn_ = 3.14;
      is_clockwise_ = true;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::LEFT:
      global_orientation_ = GlobalOrientation::NORTH;
      angle_to_turn_ = 6.28;
      is_clockwise_ = false;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::BACK:
      global_orientation_ = GlobalOrientation::WEST;
      angle_to_turn_ = 1.57;
      is_clockwise_ = false;
      return;
      break;
    default:
      break;
    }
    break;

  case GlobalOrientation::WEST:

    switch (goal_->direction)
    {
    case doogie_msgs::DoogieMoveGoal::RIGHT:
      global_orientation_ = GlobalOrientation::NORTH;
      angle_to_turn_ = 0.0;
      is_clockwise_ = true;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::LEFT:
      global_orientation_ = GlobalOrientation::SOUTH;
      angle_to_turn_ = 3.14;
      is_clockwise_ = false;
      return;
      break;
    case doogie_msgs::DoogieMoveGoal::BACK:
      global_orientation_ = GlobalOrientation::EAST;
      angle_to_turn_ = 4.71;
      is_clockwise_ = false;
      return;
      break;
    default:
      break;
    }
    break;
    
  default:
    break;
  }
}


void MoveBase::start() {
  ros::Duration rate(1 / loop_frequency_);

  ROS_INFO("Move base node has started");
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }
}

void MoveBase::loadParameters() {
  checkParameters(ph_, "distance_tolerance", &distance_tolerance_);
  checkParameters(ph_, "angle_tolerance", &angle_tolerance_);
  checkParameters(ph_, "linear_velocity", &linear_velocity_);
  checkParameters(ph_, "angular_velocity", &angular_velocity_);
  checkParameters(ph_, "loop_frequency", &loop_frequency_);
  checkParameters(ph_, "cell_size", &cell_size_);
  checkParameters(ph_, "angular_gain", &angular_gain_);
  checkParameters(ph_, "linear_gain", &linear_gain_);
  
}

// double MoveBase::computeDistanceTarget() {
//   twod_point current_point(current_pose_.position.x, current_pose_.position.y);
//   twod_point target_point(target_pose_.position.x, target_pose_.position.y);
//   double distance = boost::geometry::distance(current_point, target_point);
//   // ROS_INFO("DISTANCE = %lf", distance);
//   return distance;
// }

}  // namespace doogie_navigation
