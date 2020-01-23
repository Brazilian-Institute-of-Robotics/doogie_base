#include <ros/ros.h>
#include "doogie_perception/doogie_perception.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "doogie_perception_node");

  doogie_perception::DoogiePerception doogie_perception;
  doogie_perception.run();

  return 0;
}
