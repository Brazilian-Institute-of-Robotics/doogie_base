#ifndef DOOGIE_NAVIGATION_MOVE_BASE_HPP_
#define DOOGIE_NAVIGATION_MOVE_BASE_HPP_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <doogie_msgs/DoogieMoveAction.h>
#include <boost/geometry/geometries/point_xy.hpp>

namespace doogie_navigation {

typedef boost::geometry::model::d2::point_xy<double> twod_point;

class MoveBase {
 public:
  explicit MoveBase(const ros::NodeHandle &robot_pose_nh);
  void receiveGoalCallback();
  void preemptGoalCallback();
  void getOdometryDataCallback(const nav_msgs::Odometry::ConstPtr &odometry_data);
  void moveRobot(double distance, double angle);
  void start();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle robot_controller_nh_;
  ros::NodeHandle ph_;

  ros::Subscriber odom_sub_;
  ros::Publisher cmd_vel_pub_;

  actionlib::SimpleActionServer<doogie_msgs::DoogieMoveAction> move_to_goal_action_server_;
  doogie_msgs::DoogieMoveGoalConstPtr goal_;
  doogie_msgs::DoogieMoveResult move_to_goal_result_;
  doogie_msgs::DoogieMoveFeedback move_to_goal_feedback_;

  geometry_msgs::Twist twist_cmd_;
  geometry_msgs::Pose target_pose_;
  geometry_msgs::Pose current_pose_;

 private:
  void loadParameters();
  double computeDistanceTarget();
  double computeAngleTarget();

  bool is_to_move_straight_;
  bool is_to_turn_;

  double distance_tolerance_;
  double angle_tolerance_;
  double linear_velocity_;
  double angular_velocity_;
  double loop_frequency_;
  double cell_size_;
};

}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_BASE_HPP_
