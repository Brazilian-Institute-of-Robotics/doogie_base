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

MoveBase::MoveBase(const ros::NodeHandle& robot_nh)
: pid_{ros::NodeHandle{robot_nh, "/pid/linear"}, ros::NodeHandle{robot_nh, "/pid/angular"}}
, params(robot_nh) {}

double MoveBase::computeAngleToTurn(double angle_offset) {
  double actual_angle = diff_drive_controller.getCurrentNormalizedYawOrientation();
  angle_to_turn_ = angle_offset + actual_angle;
  if (angle_to_turn_ < 0) {
    angle_to_turn_ += (2 * doogie::PI);
  }
  return angle_to_turn_;
}

void MoveBase::turnRobot(double rad_angle) {
  double current_orientation = diff_drive_controller.getCurrentYawOrientation();
  double control_action_input = pid_[ANGULAR].computeControlAction(rad_angle, current_orientation);
  while (isTurnRobotGoalReached()) {
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

  return (pid_[ANGULAR].computeAbsoluteError(actual_orientation) <= params.angle_tolerance);
}

double MoveBase::getCurrentOrientation() {
  return diff_drive_controller.getCurrentYawOrientation();
}


void MoveBase::moveHeadingXAxis(double goal_position) {
  double setpoint = goal_position;
  pid_[LINEAR].setSetPoint(setpoint);

  double current_position = diff_drive_controller.getCurrentXPosition();

  while (!isMoveForwardGoalReached(current_position)) {
    double control_action_input = pid_[LINEAR].computeControlAction(current_position);

    diff_drive_controller.setLinearVelocity(control_action_input);
    diff_drive_controller.publishVelocityMessage();
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");

    current_position = diff_drive_controller.getCurrentXPosition();
    spin();
  }

  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Error: " << pid_[LINEAR].getAbsoluteError() );
  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Tolerance: " << params.distance_tolerance);
}

void MoveBase::moveHeadingYAxis(double goal_position) {
  double setpoint = goal_position;
  pid_[LINEAR].setSetPoint(setpoint);

  double current_position = diff_drive_controller.getCurrentYPosition();
  double control_action_input = pid_[LINEAR].computeControlAction(current_position);
  while (!isMoveForwardGoalReached(current_position)) {
    diff_drive_controller.setLinearVelocity(control_action_input);
    diff_drive_controller.publishVelocityMessage();
    current_position = diff_drive_controller.getCurrentYPosition();
    control_action_input = pid_[LINEAR].computeControlAction(current_position);
    ROS_INFO_STREAM_THROTTLE(1, "pub foward");

    spin();
  }

  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Error: " << pid_[LINEAR].getAbsoluteError() );
  ROS_DEBUG_STREAM_THROTTLE(1, "Linear Tolerance: " << params.distance_tolerance);
}

bool MoveBase::isMoveForwardGoalReached(double actual_position) {
  pid_[LINEAR].computeError(actual_position);
  ROS_DEBUG_STREAM_THROTTLE(0.5, "isMoveForwardGoalReached: ");
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Linear Error: " << pid_[LINEAR].getAbsoluteError() );
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Linear Tolerance: " << params.distance_tolerance);
  ROS_DEBUG_STREAM_THROTTLE(0.5, "distance to move: " << pid_[LINEAR].getSetPoint());

  return (pid_[LINEAR].getAbsoluteError() <= params.distance_tolerance);
}

void MoveBase::start() {
/*   ROS_DEBUG_STREAM("Start");
  ros::Rate rate(params.loop_frequency);

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
      // position_pub_.publish(robot_position_);
      robot_state_ = State::IDLE;
      break;
  }
    diff_drive_controller.publishVelocityMessage();

    ros::spinOnce();
    rate.sleep();
  } */
}

void MoveBase::spin() {
  ros::Rate rate(params.loop_frequency);
  ros::spinOnce();
  rate.sleep();
}

void MoveBase::configureControllers() {
  pid_[LINEAR].setTolerance(params.distance_tolerance);
  pid_[ANGULAR].setTolerance(params.distance_tolerance);
}

}  // namespace doogie_navigation
