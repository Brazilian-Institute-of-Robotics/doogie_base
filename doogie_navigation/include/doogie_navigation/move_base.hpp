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

#include <doogie_control/diff_driver_controller.hpp>
#include <doogie_control/odom_controller.hpp>
#include <doogie_control/pid_controller.hpp>
#include <doogie_msgs/DoogieMoveAction.h>
#include <doogie_navigation/move_base_params.hpp>
#include <doogie_navigation/maze_pose.hpp>

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

enum State {
  IDLE,
  TURNNING,
  MOVING
};

enum Heading {
    NEGATIVE=-1,
    POSITIVE=1
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
  /**
   * @brief Construct a new Move Base object
   * 
   * @param robot_namespace node handle to get the robot parameters.
   */
  explicit MoveBase(const ros::NodeHandle& robot_nh);
  /**
   * @brief The callback function executed in each goal received by the action server.
   * 
   * @param goal The goal to move_base send the robot.
   */
  void moveHeadingXAxis(double goal_position, Heading heading);
  void moveHeadingYAxis(double goal_position, Heading heading);
  void turnRobot(double rad_angle);
  virtual void start();

 protected:
  enum Error {
    LINEAR,
    ANGULAR
  };

  ros::NodeHandle nh_;

  doogie_control::PIDController pid_[2];
  doogie_control::DiffDriveController motion_iface_;
  State robot_state_;

  MoveBaseParams params_;

 private:
  void configureControllers();
  void spin();
  bool isMoveForwardGoalReached(double actual_position);
  bool isTurnRobotGoalReached();
};

}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_BASE_HPP_
