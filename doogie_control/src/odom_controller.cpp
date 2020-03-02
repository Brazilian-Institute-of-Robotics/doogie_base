#include <angles/angles.h>

#include <tf/transform_datatypes.h>

#include "doogie_control/odom_controller.hpp"

namespace doogie_control {

OdomController::OdomController() {
    odom_sub_ = nh.subscribe("odom", 1, &OdomController::odomCallBack, this);

}

OdomController::OdomController(const std::string& topic_name) : nh(topic_name) {
    odom_sub_ = nh.subscribe(topic_name, 1, &OdomController::odomCallBack, this);
}

void OdomController::initialize(const std::string& topic_name) {
    nh = ros::NodeHandle(topic_name);
    odom_sub_ = nh.subscribe(topic_name, 1, &OdomController::odomCallBack, this);
}

OdomController::~OdomController() {

}

void OdomController::odomCallBack(const nav_msgs::Odometry::ConstPtr &odometry_msg) {
  ROS_DEBUG_THROTTLE(0.5, "odom callback");
  current_pose_ = odometry_msg->pose.pose;
}

double OdomController::getCurrentXPosition() {
    return current_pose_.position.x;
}

double OdomController::getCurrentYPosition() {
    return current_pose_.position.y;
}

double OdomController::getCurrentYawOrientation() {
    return tf::getYaw(current_pose_.orientation);
}

double OdomController::getCurrentNormalizedYawOrientation() {
    return angles::normalize_angle_positive(getCurrentYawOrientation());
}

}  // namespace doogie_control
