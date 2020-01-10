#ifndef DOOGIE_ALGORITHMS_BASE_SOLVER_HPP_
#define DOOGIE_ALGORITHMS_BASE_SOLVER_HPP_

#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"

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


};

};

#endif