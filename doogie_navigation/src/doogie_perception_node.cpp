#include <ros/ros.h>
#include "doogie_navigation/doogie_perception.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "doogie_perception_node");

  doogie_navigation::DoogiePerception doogie_perception;
  doogie_perception.run();
  
  return 0;
}
