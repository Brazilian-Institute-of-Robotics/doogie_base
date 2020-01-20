#ifndef DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_
#define DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_

#include <ros/ros.h>
#include "doogie_core/base_solver.hpp"
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"

namespace doogie_algorithms{

class RHSolverPlugin : public doogie_core::BaseSolver{
  public:
    RHSolverPlugin();
    bool makePlan() override;
    bool move() override;
    void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg) override;
  
  private:
    doogie_msgs::MazeCellMultiArrayPtr matrix_maze_;
};

}

#endif