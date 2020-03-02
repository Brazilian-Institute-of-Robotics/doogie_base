#include <ros/ros.h>
#include <doogie_navigation/move_base.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_base_node");

  doogie_navigation::MoveBase move_base("doogie_move_base_controller");

  move_base.start();

  return 0;
}
