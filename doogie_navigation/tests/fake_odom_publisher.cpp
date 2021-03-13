#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <Quaternion.h>

namespace doogie_control {

class FakeOdomPublisher {
 public:
  FakeOdomPublisher() {
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &FakeOdomPublisher::cmdVelCallBack, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 5);
  }

  void start() {
    fake_odom_msg_.pose.pose = geometry_msgs::Pose();  // set everything to zero
    odom_pub_.publish(fake_odom_msg_);
  }

  
  void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr &cmd_vell_msg) {
    fake_odom_msg_ = computeFakeOdom(cmd_vell_msg);
    odom_pub_.publish(fake_odom_msg_);
  }

  nav_msgs::Odometry computeFakeOdom(geometry_msgs::Twist::ConstPtr cmd_vell_msg) {
    geometry_msgs::Vector3 angular_component = cmd_vell_msg->angular;
    tf2::Quaternion angular_velocity;
    angular_velocity.setEuler(angular_component.x, angular_component.y, angular_component.z);

    //  fake_odom_msg_.pose.pose.orientation += angular_velocity * dt;
    return fake_odom_msg_;
  }

 private:
    ros::NodeHandle nh; 
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    ros::Time current_time;

    nav_msgs::Odometry fake_odom_msg_;

};
    
}  // namespace doogie_control
