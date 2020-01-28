#ifndef DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_
#define DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_

#include <ros/ros.h>
#include "doogie_core/base_solver.hpp"
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"

namespace doogie_algorithms{

namespace right_hand_solver_plugin{


class RightHandSolverPlugin : public doogie_core::BaseSolver{
  public:
    RightHandSolverPlugin();
    bool makePlan() override;
    bool move() override;
    void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg) override;
    void mazeMatrixCallback(const doogie_msgs::MazeCellMultiArray& matrix_maze) override;
    bool isPlanAttemptsReached(int count);
  
  private:
    int replan_count;
};

} // namespace doogie_algorithms::right_hand_solver_plugin
} // namespace doogie_algorithms

#endif