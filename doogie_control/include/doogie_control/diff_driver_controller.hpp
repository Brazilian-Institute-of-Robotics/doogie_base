#ifndef DOOGIE_CONTROL_DIFF_DRIVE_CONTROLLER_HPP_
#define DOOGIE_CONTROL_DIFF_DRIVE_CONTROLLER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace doogie_control {
    
class DiffDriveController {
 public:
  DiffDriveController();
  ~DiffDriveController();
  void odomCallBack(const nav_msgs::Odometry::ConstPtr &odometry_msg);
  double getCurrentXPosition();
  double getCurrentYPosition();
  double getCurrentYawOrientation();
  double getCurrentNormalizedYawOrientation();
  void increaseLinearVelocity(double linear_velocity);
  void increaseAngularVelocity(double angular_velocity);
  void setLinearVelocity(double linear_velocity);
  void setAngularVelocity(double angular_velocity);
  void publishLinearVelocityMessage(double linear_velocity);
  void publishAngularVelocityMessage(double angular_velocity);
  void publishVelocityMessage();
  void stop();

 private:
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  geometry_msgs::Twist velocity_msg_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Twist current_velocity_;
};

}  // namespace doogie_control

#endif  // DOOGIE_CONTROL_DIFF_DRIVE_CONTROLLER_HPP_
