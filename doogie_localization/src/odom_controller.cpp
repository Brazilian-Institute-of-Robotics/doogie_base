#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

#include "doogie_localization/odom_controller.hpp"

namespace doogie_localization {

OdomController::OdomController() {
  std::string topic_name = "odom"; // change topic_name to the diff_drive_controller one
  pn_.getParam("input_topic", topic_name);
  odom_sub_ = nh_.subscribe(topic_name, 1, &OdomController::odomCallBack, this);
}

void OdomController::odomCallBack(const nav_msgs::OdometryPtr &odometry_msg) {
  ROS_DEBUG_THROTTLE(0.5, "odom callback");
  current_pose_ = odometry_msg->pose.pose;
  current_velocity_ = odometry_msg->twist.twist;
}

double OdomController::getCurrentXPosition() {
  return current_pose_.position.x;
}

double OdomController::getCurrentYPosition() {
  return current_pose_.position.y;
}

double OdomController::getCurrentYawOrientation() {
  return tf2::getYaw(current_pose_.orientation);
}

double OdomController::getCurrentNormalizedYawOrientation() {
  return angles::normalize_angle(getCurrentYawOrientation());
}

}  // namespace doogie_localization

PLUGINLIB_EXPORT_CLASS(doogie_localization::OdomController, doogie_localization::BaseLocalization)
