#include <ros/ros.h>
#include <doogie_navigation/move_base.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_base_node");

  ros::NodeHandle robot_base_nh("doogie/move_base_controller");
  doogie_navigation::MoveBase move_base(robot_base_nh);

  move_base.start();

  return 0;
}
