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
  , is_to_change_robot_state_(false)
  , current_robot_state_(STOPPED)
  , next_robot_state_(STOPPED)
  , angle_to_turn_(0) {
  move_to_goal_action_server_.registerGoalCallback(boost::bind(&MoveBase::receiveGoalCallback, this));
  move_to_goal_action_server_.registerPreemptCallback(boost::bind(&MoveBase::preemptGoalCallback, this));

  odom_sub_ = robot_controller_nh_.subscribe("odom", 1, &MoveBase::getOdometryDataCallback, this);
  cmd_vel_pub_ = robot_controller_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  this->loadParameters();
  move_to_goal_action_server_.start();
}

void MoveBase::receiveGoalCallback() {
  if (current_robot_state_ == STOPPED) {
    goal_ = move_to_goal_action_server_.acceptNewGoal();
    is_to_change_robot_state_ = true;
  }

  target_pose_.position.x = current_pose_.position.x + 0.18;
  target_pose_.orientation.w = 1.0;
  ROS_INFO("Goal received");
}

void MoveBase::preemptGoalCallback() {
  ROS_INFO("Goal id preempted!");
  move_to_goal_action_server_.setPreempted();
}

void MoveBase::getOdometryDataCallback(const nav_msgs::Odometry::ConstPtr &odometry_data) {
  current_pose_ = odometry_data->pose.pose;
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Received odometry data");
}

void MoveBase::moveRobot() {
  ros::Time start_time = ros::Time::now();
  ros::Duration rate(1.0 / loop_frequency_);

  doogie_msgs::DoogieMoveResult result;

  while(1) {
    ROS_INFO_THROTTLE(1, "Moving robot");
    float distance = this->computeDistanceTarget();
    float angle = this->computeAngleTarget();

    move_to_goal_feedback_.progress = distance;
    move_to_goal_action_server_.publishFeedback(move_to_goal_feedback_);

    if ((fabs(distance) <= distance_tolerance_) && (fabs(angle) <= angle_tolerance_)) {
      twist_cmd_.linear.x = 0;
      twist_cmd_.angular.z = 0;
      result.status = true;
      move_to_goal_action_server_.setSucceeded(result, "robot stopped");
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
    ros::spinOnce();
  }
}


void MoveBase::start() {
  ros::Duration rate(1 / loop_frequency_);

  ROS_INFO("Move base node has started");
  while (ros::ok()) {
    if (move_to_goal_action_server_.isActive()) this->updateRobotState();
    if (is_to_change_robot_state_) this->performRobotStateTask();

    ROS_INFO_THROTTLE(1, "Current state = %d", current_robot_state_);

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
  // ROS_INFO("DISTANCE = %lf", distance);
  return distance;
}

double MoveBase::computeAngleTarget() {
  double angle = tf::getYaw(current_pose_.orientation) - tf::getYaw(target_pose_.orientation);
  // ROS_INFO("ANGLE = %lf", angle);
  return angle;
}

void MoveBase::updateRobotState() {
  is_to_change_robot_state_ = true;
  switch (goal_->direction) {
    case doogie_msgs::DoogieMoveGoal::FRONT:
      next_robot_state_ = MOVING_STRAIGHT;
      angle_to_turn_ = 0;
      ROS_INFO("Robot will move front");
      return;
    case doogie_msgs::DoogieMoveGoal::BACK:
      next_robot_state_ = TURNING;
      angle_to_turn_ = M_PI;
      ROS_INFO("Robot will move back");
      return;
    case doogie_msgs::DoogieMoveGoal::LEFT:
      next_robot_state_ = TURNING;
      angle_to_turn_ = M_PI_2;
      ROS_INFO("Robot will move left");
      return;
    case doogie_msgs::DoogieMoveGoal::RIGHT:
      next_robot_state_ = TURNING;
      angle_to_turn_ = -M_PI_2;
      ROS_INFO("Robot will move right");
  }
}

void MoveBase::performRobotStateTask() {
  current_robot_state_ = next_robot_state_;

  switch (next_robot_state_) {
    case MOVING_STRAIGHT:
      ROS_INFO_THROTTLE(1, "Robot will move straight");
      target_pose_.position.x = current_pose_.position.x + cell_size_;
      target_pose_.orientation = current_pose_.orientation;

      this->moveRobot();

      ROS_INFO_THROTTLE(1, "Robot moved straight");
      is_to_change_robot_state_ = true;
      next_robot_state_ = STOPPED;

      return;
    case TURNING: {
      ROS_INFO_THROTTLE(1, "Robot will turning");
      angle_to_turn_ += tf::getYaw(current_pose_.orientation);
      
      tf::Quaternion quaternion;
      quaternion.setRPY(0, 0, angle_to_turn_);

      target_pose_.position = current_pose_.position;

      target_pose_.orientation.x = quaternion.getX();
      target_pose_.orientation.y = quaternion.getY();
      target_pose_.orientation.z = quaternion.getZ();
      target_pose_.orientation.w = quaternion.getW();

      this->moveRobot();

      is_to_change_robot_state_ = true;
      next_robot_state_ = MOVING_STRAIGHT;
      ROS_INFO_THROTTLE(1, "Robot turned");
      return;
    }
    case STOPPED: {
      ROS_INFO_THROTTLE(1, "Robot will stop");
      target_pose_ = current_pose_;
      is_to_change_robot_state_ = false;
      break;
    }
  }
}

}  // namespace doogie_navigation
