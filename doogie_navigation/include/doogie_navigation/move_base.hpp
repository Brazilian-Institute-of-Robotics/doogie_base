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

// typedef boost::geometry::model::d2::point_xy<double> twod_point;

/**
 * @class MoveBase move_base.hpp
 * @brief Class to control the doogie robot in the maze, receiving goal from the doogie_algorithms_node
 * and send the robot to the desired point. It's command the robot through the cmd_vel topic and ready
 * odometry from odom topic to control the position.
 * 
 */
class MoveBase {
 public:
  /**
   * @brief Construct a new Move Base object
   * 
   * @param robot_pose_nh node handle to get the robot parameters.
   */
  explicit MoveBase(const ros::NodeHandle &robot_pose_nh);
  /**
   * @brief The callback function executed in each goal received by the action server.
   * 
   * @param goal The goal to move_base send the robot.
   */
  void executeCB(const doogie_msgs::DoogieMoveActionGoalConstPtr &goal);
  // void receiveGoalCallback();
  // void preemptGoalCallback();
  /**
   * @brief Get the odometry data from the odometry topic to control the position of the 
   * robot.
   * 
   * @param odometry_data The data received from the topic.
   */
  void getOdometryDataCallback(const nav_msgs::Odometry::ConstPtr &odometry_data);
  void start();

 protected:
  // void moveRobot();
  
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
  /**
   * @brief Check if the desired parameter is in the parameter server. If the parameter exist, return trhough pointer,
   * throw an excepction, otherwise.
   * 
   * @tparam param_type 
   * @param private_nh Nodehandle to acess the parameter server.
   * @param param_name Name of the parameter
   * 
   * @param[out] param_value Value of the desired parameter
   */
  template <typename param_type>
  void checkParameters(const ros::NodeHandle& private_nh, const std::string& param_name, param_type* param_value) {
    if(!private_nh.hasParam(param_name)) {
      std::stringstream error_msgs;
      error_msgs << "No parameter with name:" + param_name + "was found in the parameter server.";
      ROS_ERROR_STREAM(error_msgs.str());
      throw std::runtime_error (error_msgs.str());
    }

    private_nh.getParam(param_name, *param_value);
  }
  /**
   * @brief Load all parameters from parameter server
   * 
   */
  void loadParameters();
  float angle_to_turn_;
  bool is_to_change_robot_state_;
  double distance_tolerance_;
  double angle_tolerance_;
  double linear_velocity_;
  double angular_velocity_;
  double loop_frequency_;
  double cell_size_;
};

}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_BASE_HPP_
