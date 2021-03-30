#ifndef DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_
#define DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_

#include <ros/ros.h>
#include "doogie_algorithms/base_solver.hpp"
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePose.h"

namespace doogie_algorithms {

namespace right_hand_solver_plugin {


class RightHandSolverPlugin : public BaseSolver {
 public:
  RightHandSolverPlugin() = default;
  void initialize() override;
  bool makePlan() override;
  bool move() override;
  bool isPlanAttemptsReached(int count);

 private:
  void loadParams();
  struct ROSParams : BaseSolver::ROSParams {
    int plan_attempts{5};
  } params_;
};

}  // namespace right_hand_solver_plugin
}  // namespace doogie_algorithms

#endif  // DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_