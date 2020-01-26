#ifndef DOOGIE_ALGORITHMS_ASTAR_PLUGIN_HPP_
#define DOOGIE_ALGORITHMS_ASTAR_PLUGIN_HPP_

#include <ros/ros.h>

#include "doogie_core/base_solver.hpp"
#include "doogie_core/maze_matrix_handle.hpp"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_msgs/MazeCellMultiArray.h"

namespace doogie_algorithms{

namespace astar_solver_plugin{
struct CostFromStart{
  static int calculateCost(doogie_msgs::MazeCellPtr current_cell);
  static int value;
  static doogie_msgs::MazeCell start_cell_;
};
struct CostToGoal{
  static int calculateCost(doogie_msgs::MazeCellPtr current_cell);
  static int value;
  static doogie_msgs::MazeCell goal_cell_;
};

class AStarSolverPlugin : public doogie_core::BaseSolver{
  public:
    AStarSolverPlugin();
    bool makePlan() override;
    bool move() override;
    void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg) override;
  
  protected:
    void calculateMazeCellCost(doogie_msgs::MazeCellPtr);
    void updateOpenList();
    CostFromStart g;
    CostToGoal h;

    doogie_msgs::MazeCellPtr goal_cell_;

    MazeMatrixPtr open_list_;
    MazeMatrixPtr closed_list_;
    
};

} // namespace doogie_algorithms::astar_solver_plugin namespace

} // namespace doogie_algorithms

#endif