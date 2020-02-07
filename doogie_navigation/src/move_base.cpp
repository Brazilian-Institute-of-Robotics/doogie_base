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
  , angle_to_turn_(0) {
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
  int direction = goal_->direction;
  is_to_change_robot_state_ = true;

  computeDistance(goal_->cells);
  computeAngle(direction);

  
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
  double angle = tf::getYaw(current_pose_.orientation);
  angle = angles::normalize_angle_positive(angle);
  // if(angle < 0) {
  //   angle = angle + 6.28;
  // }


  // tf::Quaternion quat;
  // double roll, pitch, yaw;
  // tf::quaternionMsgToTF(current_pose_.orientation, quat);
  // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  ROS_INFO_STREAM_THROTTLE(1, "angle");
  ROS_INFO_STREAM_THROTTLE(1, angle);

  // getOrientation(current_pose_.orientation);
  // // ROS_INFO_STREAM_THROTTLE(1, is_heading_x_);

  ROS_INFO_STREAM_THROTTLE(1, "angle to turn");
  ROS_INFO_STREAM_THROTTLE(1, angle_to_turn_);

  double error = std::abs(angle_to_turn_ - angle);
  // double error = angle_to_turn_ - angle;
  if(is_clockwise_) {
    twist_cmd_.angular.z = -0.2 * error;
  }else {
    twist_cmd_.angular.z = 0.2 * error;
  }

  // twist_cmd_.angular.z = 0.2 * error;

  ROS_INFO_STREAM_THROTTLE(1, "error");
  ROS_INFO_STREAM_THROTTLE(1, error);

  if(is_to_change_robot_state_) {

    if (std::abs(error) >= angle_tolerance_) {
      ROS_INFO_STREAM_THROTTLE(1, "pub");
      cmd_vel_pub_.publish(twist_cmd_);
    }else if(error < angle_tolerance_) {
      is_to_change_robot_state_ = false;

    }
  
  }
  
}
void MoveBase::getOrientation(const geometry_msgs::Quaternion& orientation) {
  
  double yaw_angle = tf::getYaw(orientation);
  
  if ( (1.57 - angle_tolerance_) <= std::abs(yaw_angle) && 
       (1.57 + angle_tolerance_) >= std::abs(yaw_angle) ) {
    is_heading_x_ = false;
    return;
  }

  is_heading_x_ = true;
  return;
}

void MoveBase::computeDistance(int cell_number) {
    distance_to_move_ = cell_number * cell_size_;
}

void MoveBase::computeAngle(int direction) {
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
  
}

// double MoveBase::computeDistanceTarget() {
//   twod_point current_point(current_pose_.position.x, current_pose_.position.y);
//   twod_point target_point(target_pose_.position.x, target_pose_.position.y);
//   double distance = boost::geometry::distance(current_point, target_point);
//   // ROS_INFO("DISTANCE = %lf", distance);
//   return distance;
// }

}  // namespace doogie_navigation
