#ifndef DOOGIE_NAVIGATION_MOVE_BASE_HPP_
#define DOOGIE_NAVIGATION_MOVE_BASE_HPP_

#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>

#include <doogie_control/pid_controller.hpp>
#include <doogie_control/diff_driver_controller.hpp>
#include <doogie_msgs/DoogieMoveAction.h>
#include <doogie_navigation/move_base_params.hpp>
#include <doogie_navigation/maze_pose.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

/**
 * TODO 
 * 
 * 2- remove comments and LOG_INFO debug msgs
 * 
 * 4- Update the global_orientation_ cheking if is_heading_x_ was changed
 * 5- Corrigir computeAngle para resolver o problema da mudança de 2pi pra 0
 * 6- Colocar updateOrientation so depois que o robo chegar na posicao, separar
 * o metodo para setar posicao absoluta
 * 7- Robo nao funciona corretamente quando reinicia o launch com a simulacao
 * funcionando
 * 8- Colocar valor de matrix como default?
 *  
 * 
 */

namespace doogie {
  constexpr double PI = 3.14159265358979323846;
}  // namespace doogie

namespace doogie_navigation {

// typedef boost::geometry::model::d2::point_xy<double> twod_point;

enum GlobalOrientation : int8_t {
  NORTH = doogie_msgs::DoogieOrientation::NORTH,
  SOUTH = doogie_msgs::DoogieOrientation::SOUTH,
  EAST = doogie_msgs::DoogieOrientation::EAST,
  WEST = doogie_msgs::DoogieOrientation::WEST
};

enum State {
  IDLE,
  TURNNING,
  MOVING
};

/**
 * @class MoveBase move_base.hpp
 * @brief Class to control the doogie robot in the maze, receiving goal from the doogie_algorithms_node
 * and send the robot to the desired point. It's command the robot through the cmd_vel topic and ready
 * odometry from odom topic to control the position.
 * 
 */
class MoveBase {
 public:
 using Pose = doogie::doogie_navigation::MazePose;
  /**
   * @brief Construct a new Move Base object
   * 
   * @param robot_namespace node handle to get the robot parameters.
   */
  explicit MoveBase(const std::string &robot_namespace, const ros::NodeHandle& robot_nh);
  /**
   * @brief The callback function executed in each goal received by the action server.
   * 
   * @param goal The goal to move_base send the robot.
   */
  void moveForward();
  void moveForward(int number_of_cells);
  void moveHeadingXAxis(double goal_position);
  void moveHeadingYAxis(double goal_position);
  bool turnRobot(bool is_clockwise);
  void turnRobot(double rad_angle);
  virtual void receiveGoalCallback();
  virtual void start();

 protected:
  enum Error {
    LINEAR,
    ANGULAR
  };

  enum Direction  : int8_t {
    FRONT = doogie_msgs::DoogieMoveGoal::FRONT,
    BACK = doogie_msgs::DoogieMoveGoal::BACK,
    LEFT = doogie_msgs::DoogieMoveGoal::LEFT,
    RIGHT = doogie_msgs::DoogieMoveGoal::RIGHT
  };

  ros::NodeHandle nh_;

  ros::Publisher position_pub_;
  actionlib::SimpleActionServer<doogie_msgs::DoogieMoveAction> move_to_goal_action_server_;

  doogie_msgs::DoogieMoveGoalConstPtr goal_;
  doogie_msgs::DoogieMoveResult move_to_goal_result_;
  doogie_msgs::DoogieMoveFeedback move_to_goal_feedback_;

  doogie_msgs::DoogiePosition robot_position_;
  doogie_navigation::GlobalOrientation global_orientation_{doogie_navigation::NORTH};

  doogie_control::PIDController pid_[2];
  doogie_control::DiffDriveController diff_drive_controller;
  State robot_state_;

  MoveBaseParams params; 

 private:
  /**
   * @brief Check if whether robot is heading x or y axis. It returns true if robot is heading x axis and false otherwise. First it gets the orientation of doogie robot according to the odometry. Then the yaw angle 
   * get from the orientation using tf::getYaw() varies [0, PI] and [0, -PI]. So, if the angle is
   * equal to PI/2 or -PI/2, the robot is heading the y axis. If is equal to 0, PI or -PI is heading
   * the x axis. Considering this angle, the flag is_heading_x is set. 
   */
  // TODO correct documentation

  void configureControllers();
  void spin();

  double computeAngleToTurn(double angle_offset);
  double computeAngleToTurn(Direction goal_direction);

  double computeDistanceToMove();
  double computeLinearControlAction();
  double getCurrentPosition();
  double getCurrentOrientation();

  /**
   * @brief Check if whether robot is heading x or y axis. It returns true if robot is heading x axis and false otherwise.
   */
  bool isHeadingXAxis();

  bool isMoveForwardGoalReached();
  bool isTurnRobotGoalReached();
  
  void updatePosition();

  void updateOrientation();

  // double getCurrentRadOrientation();
  // double getCurrentNormalizedRadOrientation();

  double angle_to_turn_;
  /** Flag to set if the rotational movement will be clockwise or couter clockwise
   *    - true: clockwise
   *    - false: couter clockwise  **/
  bool is_clockwise_;
  
  int current_row_;
  int current_column_;

};

}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_BASE_HPP_
