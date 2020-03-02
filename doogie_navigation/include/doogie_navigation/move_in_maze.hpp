#ifndef DOOGIE_NAVIGATION_MOVE_IN_MAZE_HPP_
#define DOOGIE_NAVIGATION_MOVE_IN_MAZE_HPP_

#include <string>
#include <map>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

#include "doogie_control/pid_controller.hpp"
#include "doogie_control/diff_driver_controller.hpp"
#include "doogie_navigation/move_base.hpp"
#include "doogie_msgs/DoogieMoveAction.h"
#include "doogie_msgs/DoogiePose.h"

namespace doogie {

  const int8_t static computeResultDirection (int8_t const& current_orientation, int8_t const& goal_direction) {
    int8_t temp = current_orientation + goal_direction;
    if (temp > 4) {
      temp -= 4;
    } 
    if (temp < 1) {
      temp += 4;
    }
    const int8_t result = temp;
    return result;
}

}  // namespace doogie

namespace doogie_navigation {

/**
 * @class MoveInMaze MOVE_TO_CELL.hpp
 * @brief Class to control the doogie robot in the maze, receiving goal from the doogie_algorithms_node
 * and send the robot to the desired point. It's command the robot through the cmd_vel topic and ready
 * odometry from odom topic to control the position.
 * 
 */
class MoveInMaze : public MoveBase {
 public:

  explicit MoveInMaze(const std::string &robot_namespace);
  
  doogie_msgs::DoogiePosition getCurrentDoogiePosition();
  const int8_t getCurrentDoogieOrientation() const;
  void setCurrentDoogieOrientation(int8_t current_orientation);
  void moveStraight();
  void rotate();
  
  /**
   * @brief The callback function executed in each goal received by the action server.
   * 
   * @param goal The goal to MOVE_TO_CELL send the robot.
   */
  void executeCB(const doogie_msgs::DoogieMoveActionGoalConstPtr &goal);
  void receiveGoalCallback();

  /**
   * @brief Get the odometry data from the odometry topic to control the position of the 
   * robot.
   * 
   * @param odometry_data The data received from the topic.
   */
  void getOdometryDataCallback(const nav_msgs::Odometry::ConstPtr &odometry_data);
  void start() override;

 protected:
  class DoogieMoveBaseServer {
   public:
    explicit DoogieMoveBaseServer(ros::NodeHandle nh);
    ~DoogieMoveBaseServer();
    void acceptNewGoal();
    const int8_t getCurrentGoalDirection() const;
    const int8_t getCurrentGoalNumberOfCells() const;
    const bool getCurrentGoalStatus() const;
    void setSucceeded();

   private:
    actionlib::SimpleActionServer<doogie_msgs::DoogieMoveAction> move_base_server_controller_;

    doogie_msgs::DoogieMoveGoalPtr current_goal_;
    doogie_msgs::DoogieMoveResultPtr current_goal_result_;
    doogie_msgs::DoogieMoveFeedbackPtr current_goal_feedback_;
  }; 

  ros::NodeHandle nh_;
  ros::NodeHandle ph_;

  ros::Publisher position_pub_;

  doogie_msgs::DoogiePose robot_pose_;

  DoogieMoveBaseServer move_base_action_server;
  std::map<int8_t, std::function<void(const double&)>> command_map;

 private:
  
  void initializeCommandMap();
  void spin();
  double computeDistanceToMove();
  bool isHeadingXAxis();
  
  void sendRotateCommand(const double& goal_direction);
  void sendMoveStraightCommand(const int& number_of_cells);
  void sendMoveToNorthCommand(const double& distance_to_move);
  void sendMoveToSouthCommand(const double& distance_to_move);
  void sendMoveToEastCommand(const double& distance_to_move);
  void sendMoveToWestCommand(const double& distance_to_move);

  bool isRotateGoalReached();

  void updatePosition();
  void updateOrientation();

};

}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_IN_MAZE_HPP_
