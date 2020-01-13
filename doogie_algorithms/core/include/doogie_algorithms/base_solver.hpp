#ifndef DOOGIE_ALGORITHMS_BASE_SOLVER_HPP_
#define DOOGIE_ALGORITHMS_BASE_SOLVER_HPP_

#include <actionlib/client/action_client.h>
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_algorithms/mouse_handle.hpp"
#include "doogie_algorithms/matrix_handle.hpp"
namespace doogie_algorithms
{

class BaseSolver
{
  public:
    virtual void init() = 0;
    virtual void mazeWallsCallback(const doogie_msgs::MazeCellMultiArray& matrix_msg) = 0;
    virtual void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg) = 0;
    virtual ~BaseSolver(){}

  protected:
    BaseSolver(){}
    doogie_algorithms::MouseHandle doogie_handle_;
    doogie_algorithms::MatrixHandle maze_handle_;
};

};

#endif