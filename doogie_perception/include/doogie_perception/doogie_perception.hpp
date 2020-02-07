#ifndef DOOGIE_NAVIGATION_DOOGIE_PERCEPTION_HPP_
#define DOOGIE_NAVIGATION_DOOGIE_PERCEPTION_HPP_

#include <ros/ros.h>
#include "doogie_core/maze_matrix_handle.hpp"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_msgs/WallDistances.h"

namespace doogie_perception {

/**
 * @brief Class to wrapper the Doogie Percetion node.
 * 
 */
class DoogiePerception {
 public:
  /**
   * @brief Construct a new DoogiePerception object.
   * 
   */
  DoogiePerception();
  /**
   * @brief Run the DoogiePerception node.
   * 
   */
  void run();

 private:
  /**
   * @brief Callback method called when a @p doogie_position message arrives in the topic.
   * 
   * @param doogie_position Message holding robot position (row, column, orientation).
   */
  void doogiePositionCallback(const doogie_msgs::DoogiePositionConstPtr& doogie_position);
  /**
   * @brief Callback method called when a @p wall_distances message arrives in the topic.
   * 
   * @param wall_distances Message holding the distances from the robot to cell walls.
   */
  void wallDistancesCallback(const doogie_msgs::WallDistancesConstPtr& wall_distances);
  /**
   * @brief Load all parameters of the node.
   * 
   */
  void loadParameters();

  /** Global Nodehandle. */
  ros::NodeHandle nh_;
  /** Publisher to publish maze obstacle matrix. */
  ros::Publisher maze_obstacle_matrix_pub_;
  /** Subscriber to get the robot position. */
  ros::Subscriber doogie_position_sub_;
  /** Subscriber to get distances from the robot to cell walls. */
  ros::Subscriber wall_distances_sub_;

  /** Object to make operation in the maze obstacle matrix. */
  doogie_core::MazeMatrixHandle maze_obstacle_matrix_;
  /** Object to holding the last robot position published. */
  doogie_msgs::DoogiePosition doogie_position_;

  /** Flag to dictate when the maze obstacle matrix should be published. */
  bool is_to_pub_maze_obstacle_matrix_;
  /** Variable to store the threshold value used to infer if there is a wall on
   the front of the robot. */
  float front_dist_threshold_;
  /** Variable to store the threshold value used to infer if there are walls on
   the sides of the robot. */
  float side_dist_threshold_;
};

}

#endif  // DOOGIE_PERCEPTION_DOOGIE_PERCEPTION_HPP_
