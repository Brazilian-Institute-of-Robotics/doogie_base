#include <ros/ros.h>
#include <doogie_navigation/move_in_maze.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_base_node");
  ros::NodeHandle nh;

  doogie_navigation::MoveInMaze move_base(nh);

  move_base.start();

  return 0;
}
