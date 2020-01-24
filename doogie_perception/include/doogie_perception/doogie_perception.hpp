#ifndef DOOGIE_NAVIGATION_DOOGIE_PERCEPTION_HPP_
#define DOOGIE_NAVIGATION_DOOGIE_PERCEPTION_HPP_

#include <ros/ros.h>
#include "doogie_core/maze_matrix_handle.hpp"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_msgs/WallDistances.h"

namespace doogie_perception {

class DoogiePerception {
 public:
  DoogiePerception();
  void run();

 private:
  void doogiePositionCallback(const doogie_msgs::DoogiePositionConstPtr& doogie_position);
  void wallDistancesCallback(const doogie_msgs::WallDistancesConstPtr& wall_distances);
  void loadParameters();

  ros::NodeHandle nh_;
  ros::Publisher maze_obstacle_matrix_pub_;
  ros::Subscriber doogie_position_sub_;
  ros::Subscriber wall_distances_sub_;

  doogie_core::MazeMatrixHandle maze_obstacle_matrix_;
  doogie_msgs::DoogiePosition doogie_position_;

  bool is_to_pub_maze_obstacle_matrix_;
  float front_dist_threshold_;
  float side_dist_threshold_;
};

}

#endif  // DOOGIE_PERCEPTION_DOOGIE_PERCEPTION_HPP_
