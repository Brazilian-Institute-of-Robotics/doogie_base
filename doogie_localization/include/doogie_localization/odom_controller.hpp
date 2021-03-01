#ifndef DOOGIE_LOCALIZATION_ODOM_CONTROLLER_HPP_
#define DOOGIE_LOCALIZATION_ODOM_CONTROLLER_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


#include "doogie_localization/base_localization.hpp"

namespace doogie_localization {
    
class OdomController : public BaseLocalization {
 public:
  OdomController();
  virtual ~OdomController() = default;
  void odomCallBack(const nav_msgs::OdometryPtr &odometry_msg);
  double getCurrentXPosition() override;
  double getCurrentYPosition() override;
  double getCurrentYawOrientation() override;
  double getCurrentNormalizedYawOrientation() override;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pn_{"~"};
  ros::Subscriber odom_sub_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Twist current_velocity_;
  
};

}  // namespace doogie_localization

#endif  // DOOGIE_LOCALIZATION_ODOM_CONTROLLER_HPP_
