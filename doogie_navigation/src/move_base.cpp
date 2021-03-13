#include "doogie_navigation/move_base.hpp"

#include <cmath>
#include <string>

#include <angles/angles.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace doogie_navigation {

using doogie_msgs::DoogiePosition;
using doogie_msgs::DoogieOrientation;

const std::string& BASE_CLASS_PACKAGE = "doogie_localization";
const std::string& BASE_CLASS = BASE_CLASS_PACKAGE + "::BaseLocalization";

MoveBase::MoveBase(const ros::NodeHandle& robot_nh)
: pid_{ros::NodeHandle{robot_nh, "pid/linear"}, ros::NodeHandle{robot_nh, "pid/angular"}}
, params_(robot_nh) {
  localization_iface_ =
    pluginlib::ClassLoader<doogie_localization::BaseLocalization>{BASE_CLASS_PACKAGE, BASE_CLASS}
      .createInstance(params_.localization_plugin);
}

void MoveBase::turnRobot(double rad_angle) {
  double target_angle = angles::normalize_angle(rad_angle);
  double current_orientation =
    angles::normalize_angle(localization_iface_->getCurrentNormalizedYawOrientation());
  double control_action_input = pid_[ANGULAR].computeControlAction(target_angle, current_orientation);
  ROS_INFO_STREAM("turning");
  while (!isTurnRobotGoalReached()) {
    motion_iface_.setAngularVelocity(control_action_input);
    motion_iface_.publishVelocityMessage();
    current_orientation = 
      angles::normalize_angle(localization_iface_->getCurrentNormalizedYawOrientation());
    control_action_input = pid_[ANGULAR].computeControlAction(target_angle, current_orientation);
    ROS_INFO_STREAM_THROTTLE(1, "\nstill rotating");

    spin();
  }
  ROS_INFO_STREAM("\n--------------Goal is finished--------------\n");
}

bool MoveBase::isTurnRobotGoalReached() {
  double actual_orientation = angles::normalize_angle(localization_iface_->getCurrentNormalizedYawOrientation());

  return (pid_[ANGULAR].getAbsoluteError() <= params_.angle_tolerance);
}

void MoveBase::moveHeadingXAxis(double goal_position, Heading heading) {
  double setpoint = goal_position;
  pid_[LINEAR].setSetPoint(setpoint);

  double current_position = localization_iface_->getCurrentXPosition();

  while (!isMoveForwardGoalReached(current_position)) {
    double control_action_input = pid_[LINEAR].computeControlAction(current_position);

    motion_iface_.setLinearVelocity(heading * control_action_input);
    motion_iface_.publishVelocityMessage();
    current_position = localization_iface_->getCurrentXPosition();
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");

    spin();
  }
  ROS_INFO_STREAM("\n--------------Goal is finished--------------\n");
}

void MoveBase::moveHeadingYAxis(double goal_position, Heading heading) {
  double setpoint = goal_position;
  pid_[LINEAR].setSetPoint(setpoint);

  double current_position = localization_iface_->getCurrentYPosition();
  double control_action_input = pid_[LINEAR].computeControlAction(current_position);
  while (!isMoveForwardGoalReached(current_position)) {
    motion_iface_.setLinearVelocity(heading * control_action_input);
    motion_iface_.publishVelocityMessage();
    current_position = localization_iface_->getCurrentYPosition();
    control_action_input = pid_[LINEAR].computeControlAction(current_position);
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");

    spin();
  }
  ROS_INFO_STREAM("\n--------------Goal is finished--------------\n");
}

bool MoveBase::isMoveForwardGoalReached(double actual_position) {
  pid_[LINEAR].computeError(actual_position);

  return (pid_[LINEAR].getAbsoluteError() <= params_.distance_tolerance);
}

void MoveBase::start() {
  ROS_ERROR_STREAM("THIS SHOULD NEVER BE PRINTED");
/*   ROS_DEBUG_STREAM("Start");
  ros::Rate rate(params_.loop_frequency);

  ROS_INFO("Move base node has started");
  while (ros::ok()) {
  switch (robot_state_) {
    case State::IDLE:
      motion_iface_.stop();
      ROS_INFO_STREAM_THROTTLE(1, "idle");
      break;

    case State::TURNNING:
      ROS_INFO_STREAM_THROTTLE(1, "turning");
      if (!turnRobot(is_clockwise_)) {
        motion_iface_.stop();
        robot_state_ = State::MOVING;
      }
      break;

    case State::MOVING:
      ROS_INFO_STREAM_THROTTLE(1, "moving");
      moveForward();
      motion_iface_.stop();
      // pose_pub_.publish(robot_position_);
      robot_state_ = State::IDLE;
      break;
  }
    motion_iface_.publishVelocityMessage();

    ros::spinOnce();
    rate.sleep();
  } */
}

void MoveBase::spin() {
  ros::Rate rate(params_.loop_frequency);
  ros::spinOnce();
  rate.sleep();
}

void MoveBase::configureControllers() {
  pid_[LINEAR].setTolerance(params_.distance_tolerance);
  pid_[ANGULAR].setTolerance(params_.angle_tolerance);
}

}  // namespace doogie_navigation
