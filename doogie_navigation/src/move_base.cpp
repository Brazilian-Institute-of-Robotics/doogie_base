#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <string>

#include "doogie_navigation/move_base.hpp"
#include <boost/geometry/algorithms/distance.hpp>

namespace doogie_navigation {

MoveBase::MoveBase(const ros::NodeHandle &robot_controller_nh)
  : move_to_goal_action_server_(nh_, "move_base_action_server", false)
  , ph_("~")
  , robot_controller_nh_(robot_controller_nh)
  , is_to_move_straight_(false)
  , is_to_turn_(false) {
  move_to_goal_action_server_.registerGoalCallback(boost::bind(&MoveBase::receiveGoalCallback, this));
  move_to_goal_action_server_.registerPreemptCallback(boost::bind(&MoveBase::preemptGoalCallback, this));

  odom_sub_ = robot_controller_nh_.subscribe("odom", 1, &MoveBase::getOdometryDataCallback, this);
  cmd_vel_pub_ = robot_controller_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  this->loadParameters();
  move_to_goal_action_server_.start();
}

void MoveBase::receiveGoalCallback() {
  goal_ = move_to_goal_action_server_.acceptNewGoal();

  switch (goal_->direction) {
    case doogie_msgs::DoogieMoveGoal::FRONT:
      is_to_move_straight_ = true;
      ROS_DEBUG_STREAM("Received new goal. Direction - FRONT");
      break;
    case doogie_msgs::DoogieMoveGoal::BACK:
      is_to_turn_ = true;
      ROS_DEBUG_STREAM("Received new goal. Direction - BACK");
      break;
    case doogie_msgs::DoogieMoveGoal::LEFT:
      is_to_turn_ = true;
      ROS_DEBUG_STREAM("Received new goal. Direction - LEFT");
      break;
    case doogie_msgs::DoogieMoveGoal::RIGHT:
      is_to_turn_ = true;
      ROS_DEBUG_STREAM("Received new goal. Direction - RIGHT");
  }

  target_pose_.position.x = current_pose_.position.x + 0.18;
}

void MoveBase::preemptGoalCallback() {
  ROS_INFO("Goal id preempted!");
  move_to_goal_action_server_.setPreempted();
}

void MoveBase::getOdometryDataCallback(const nav_msgs::Odometry::ConstPtr &odometry_data) {
  current_pose_ = odometry_data->pose.pose;
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Received odometry data");
}

void MoveBase::moveRobot(double distance, double angle) {
  move_to_goal_feedback_.progress = distance / cell_size_;
  move_to_goal_action_server_.publishFeedback(move_to_goal_feedback_);

  if ((fabs(distance) <= distance_tolerance_) && (fabs(angle) <= angle_tolerance_)) {
    twist_cmd_.linear.x = 0;
    twist_cmd_.angular.z = 0;
    move_to_goal_action_server_.setSucceeded();
    return;
  }

  if (fabs(distance) > distance_tolerance_)
    twist_cmd_.linear.x = linear_velocity_;
  else
    twist_cmd_.linear.x = 0;

  if (fabs(angle) > angle_tolerance_)
    twist_cmd_.angular.z = angular_velocity_;
  else
    twist_cmd_.angular.z = 0;

  cmd_vel_pub_.publish(twist_cmd_);
}


void MoveBase::start() {
  ros::Duration rate(1 / loop_frequency_);

  ROS_INFO("Move base node has started");
  while (ros::ok()) {
    if (move_to_goal_action_server_.isActive()) {
        float distance_target = this->computeDistanceTarget();
        float angle_target = this->computeAngleTarget();
        this->moveRobot(distance_target, angle_target);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void MoveBase::loadParameters() {
  std::string param_name = "distance_tolerance";
  if (!ph_.param<double>(param_name, distance_tolerance_, 0.01)) {
    ROS_WARN_STREAM(param_name + " parameter not found in the parameter server. Using default parameter of "
    << distance_tolerance_ << " m");
  }

  param_name = "angle_tolerance";
  if (!ph_.param<double>(param_name, angle_tolerance_, 0.01)) {
    ROS_WARN_STREAM(param_name + " parameter not found in the parameter server. Using default parameter of "
    << angle_tolerance_ << " m");
  }

  param_name = "linear_velocity";
  if (!ph_.param<double>(param_name, linear_velocity_, 0.5)) {
    ROS_WARN_STREAM(param_name + " parameter not found in the parameter server. Using default parameter of "
    << linear_velocity_ << " m/s");
  }

  param_name = "angular_velocity";
  if (!ph_.param<double>(param_name, angular_velocity_, 4 * M_PI)) {
    ROS_WARN_STREAM(param_name + " parameter not found in the parameter server. Using default parameter of "
    << angular_velocity_ << " rad/s");
  }

  param_name = "loop_frequency";
  if (!ph_.param<double>(param_name, loop_frequency_, 20)) {
    ROS_WARN_STREAM(param_name + " parameter not found in the parameter server. Using default parameter of "
    << loop_frequency_ << " Hz");
  }

  param_name = "cell_size";
  if (!ph_.param<double>(param_name, cell_size_, 0.18)) {
    ROS_WARN_STREAM(param_name + " parameter not found in the parameter server. Using default parameter of "
    << cell_size_ << + " m");
  }
}

double MoveBase::computeDistanceTarget() {
  twod_point current_point(current_pose_.position.x, current_pose_.position.y);
  twod_point target_point(target_pose_.position.x, target_pose_.position.y);
  double distance = boost::geometry::distance(current_point, target_point);
  ROS_INFO("DISTANCE = %lf", distance);
  return distance;
}

double MoveBase::computeAngleTarget() {
  double angle = tf::getYaw(current_pose_.orientation) - tf::getYaw(target_pose_.orientation);
  return angle;
}

}  // namespace doogie_navigation
