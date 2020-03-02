#include <angles/angles.h>

#include <tf/transform_datatypes.h>

#include "doogie_control/diff_driver_controller.hpp"

namespace doogie_control {

DiffDriveController::DiffDriveController() : nh("diff_drive_controller") {
  odom_sub_ = nh.subscribe("odom", 1, &DiffDriveController::odomCallBack, this);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  velocity_msg_.linear.x = 0;
  velocity_msg_.angular.z = 0;
}

DiffDriveController::~DiffDriveController() {
    
}

void DiffDriveController::odomCallBack(const nav_msgs::Odometry::ConstPtr &odometry_msg) {
  ROS_DEBUG_THROTTLE(1, "odom callback");
  current_pose_ = odometry_msg->pose.pose;
  current_velocity_ = odometry_msg->twist.twist;
}

double DiffDriveController::getCurrentXPosition() {
  return current_pose_.position.x;
}

double DiffDriveController::getCurrentYPosition() {
  return current_pose_.position.y;
}

double DiffDriveController::getCurrentYawOrientation() {
  return tf::getYaw(current_pose_.orientation);
}

double DiffDriveController::getCurrentNormalizedYawOrientation() {
  return angles::normalize_angle_positive(getCurrentYawOrientation());
}

void DiffDriveController::setLinearVelocity(double velocity) {
  velocity_msg_.linear.x = velocity;
}

void DiffDriveController::increaseLinearVelocity(double velocity) {
  velocity_msg_.linear.x += velocity;
}

void DiffDriveController::setAngularVelocity(double velocity) {
  velocity_msg_.angular.z = velocity;
}

void DiffDriveController::increaseAngularVelocity(double velocity) {
  velocity_msg_.angular.z += velocity;
}

void DiffDriveController::publishLinearVelocityMessage(double linear_velocity) {
  velocity_msg_.linear.x = linear_velocity;
  cmd_vel_pub_.publish(velocity_msg_);
}

void DiffDriveController::publishAngularVelocityMessage(double angular_velocity) {
  velocity_msg_.angular.z = angular_velocity;
  cmd_vel_pub_.publish(velocity_msg_);
}

void DiffDriveController::publishVelocityMessage() {
  cmd_vel_pub_.publish(velocity_msg_);
}

void DiffDriveController::stop() {
    publishLinearVelocityMessage(0);
    publishAngularVelocityMessage(0);
    
    double current_angular_velocity =  current_velocity_.angular.z;
    double current_linear_velocity =  current_velocity_.linear.x;

    while (std::abs(current_angular_velocity) >= 0.0001) {
      ROS_DEBUG_STREAM_THROTTLE(0.25, "A: current_linear_velocity: " << current_linear_velocity);
      ROS_DEBUG_STREAM_THROTTLE(0.25, "A: ccurrent_angular_velocity: " << current_angular_velocity);
      // setAngularVelocity(-0.55 * current_angular_velocity);
      // publishVelocityMessage();
      publishLinearVelocityMessage(0);
      publishAngularVelocityMessage(0);
      ros::spinOnce();
      current_angular_velocity =  current_velocity_.angular.z;
      current_linear_velocity =  current_velocity_.linear.x;
      ROS_DEBUG_STREAM_THROTTLE(0.25, "B: ccurrent_linear_velocity: " << current_linear_velocity);
      ROS_DEBUG_STREAM_THROTTLE(0.25, "B: ccurrent_angular_velocity: " << current_angular_velocity);

    }
    ROS_DEBUG_STREAM_THROTTLE(0.25, "stopped: " << current_linear_velocity << " and " << current_angular_velocity);
}

}  // namespace doogie_control
