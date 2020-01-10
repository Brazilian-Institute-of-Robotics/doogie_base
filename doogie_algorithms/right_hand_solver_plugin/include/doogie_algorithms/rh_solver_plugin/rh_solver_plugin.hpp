#ifndef DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_
#define DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_

#include <ros/ros.h>
#include "doogie_algorithms/base_solver.hpp"
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"

namespace doogie_algorithms{

class RHSolverPlugin : public BaseSolver{
  public:
    void init() override;
    void mazeWallsCallback(const doogie_msgs::MazeCellMultiArray& matrix_msg);
    void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg);

  private:
    doogie_algorithms::MouseHandle doogie_handle_;
    doogie_algorithms::LocalCell current_cell_;
    doogie_msgs::MazeCellMultiArrayPtr matrix_maze_;
    doogie_algorithms::MatrixHandle matrix_handle_;
    ros::NodeHandle nh_;
    ros::Subscriber maze_walls_sub_;
    ros::Subscriber doogie_position_sub_;
};

}

#endif