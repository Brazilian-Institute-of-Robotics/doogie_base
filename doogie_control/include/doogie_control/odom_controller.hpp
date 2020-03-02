#ifndef DOOGIE_CONTROL_ODOM_CONTROLLER_HPP_
#define DOOGIE_CONTROL_ODOM_CONTROLLER_HPP_

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace doogie_control {
    
class OdomController{

 public:
  OdomController();
  explicit OdomController(const std::string& topic_name);
  ~OdomController();
  void initialize(const std::string& topic_name);
  void odomCallBack(const nav_msgs::Odometry::ConstPtr &odometry_msg);
  double getCurrentXPosition();
  double getCurrentYPosition();
  double getCurrentYawOrientation();
  double getCurrentNormalizedYawOrientation();

 private:
  ros::NodeHandle nh{"odom"};
  ros::Subscriber odom_sub_;
  geometry_msgs::Pose current_pose_;


};

}  // namespace doogie_control

#endif  // DOOGIE_CONTROL_ODOM_CONTROLLER_HPP_
