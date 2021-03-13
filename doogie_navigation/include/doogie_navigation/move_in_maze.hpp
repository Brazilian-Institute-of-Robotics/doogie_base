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

#include "doogie_msgs/DoogieMoveAction.h"

#include "doogie_navigation/move_base.hpp"
#include "doogie_navigation/maze_goal.hpp"
#include "doogie_navigation/maze_pose.hpp"

namespace doogie_navigation {

/**
 * @class MoveInMaze MOVE_TO_CELL.hpp
 * @brief Class to control the doogie robot in the maze, receiving goal from the doogie_algorithms_node
 * and send the robot to the desired point. It commands the robot through the cmd_vel topic and updates
 * the position based on the robot's odometry.
 * 
 */
class MoveInMaze : public MoveBase {
 public:
  using Direction = doogie::Direction;
  using GlobalOrientation = doogie::GlobalOrientation;
  using Pose = doogie_navigation::MazePose;


  explicit MoveInMaze(const ros::NodeHandle& robot_nh);
  
  void moveStraight(int8_t number_of_cells);
  void rotate(Direction goal_direction);
  
  /**
   * @brief The callback function executed in each goal received by the action server.
   * 
   * @param goal The goal to MOVE_TO_CELL send the robot.
   */
  void receiveGoalCallback();
  void start() override;

 protected:
  ros::NodeHandle nh_;

  ros::Publisher pose_pub_;

  Pose robot_pose_;

  actionlib::SimpleActionServer<doogie_msgs::DoogieMoveAction> move_base_action_server;
  MazeGoal goal_;

  std::map<int8_t, std::function<void(const double&)>> command_map;

 private:
  void initializeCommandMap();
  void spin();
  
  void sendMoveToNorthCommand(double distance_to_move);
  void sendMoveToSouthCommand(double distance_to_move);
  void sendMoveToEastCommand(double distance_to_move);
  void sendMoveToWestCommand(double distance_to_move);

  void updatePosition(int cells);
  void updateOrientation(Direction goal_direction);

};

}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_IN_MAZE_HPP_
