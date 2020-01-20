#ifndef DOOGIE_CORE_BASE_SOLVER_HPP_
#define DOOGIE_CORE_BASE_SOLVER_HPP_

#include <actionlib/client/action_client.h>
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_core/mouse_handle.hpp"
#include "doogie_core/matrix_handle.hpp"
namespace doogie_core{

class BaseSolver{
  public:

    BaseSolver();
    virtual bool getParams();
    virtual bool isWallFront();
    virtual bool isWallLeft();
    virtual bool isWallRight();
    virtual bool makePlan() = 0;
    virtual bool move() = 0;
    virtual void sleep();
    virtual void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg) = 0;
    virtual ~BaseSolver(){}

  protected:
    doogie_core::LocalCell current_cell_;
    doogie_msgs::DoogieMoveGoal goal_;
    doogie_core::MouseHandle doogie_handle_;
    doogie_core::MatrixHandle matrix_handle_;
    ros::Subscriber doogie_position_sub_;
    ros::NodeHandle nh_;
    ros::Rate rate_;
};

};

#endif