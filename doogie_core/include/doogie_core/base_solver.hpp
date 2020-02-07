#ifndef DOOGIE_CORE_BASE_SOLVER_HPP_
#define DOOGIE_CORE_BASE_SOLVER_HPP_

#include <string>

#include <actionlib/client/action_client.h>

#include "doogie_msgs/DoogieMoveGoal.h"
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_core/mouse_handle.hpp"
#include "doogie_core/maze_matrix_handle.hpp"
namespace doogie_core{

class BaseSolver{
  public:

    BaseSolver();
    BaseSolver(const MazeMatrixPtr maze_matrix);
    virtual void initialize();
    virtual void configureSolverFromParams();
    virtual void waitForStart();
    virtual bool isWallFront();
    virtual bool isWallLeft();
    virtual bool isWallRight();
    virtual bool makePlan() = 0;
    virtual bool move() = 0;
    virtual void sleep();
    virtual void setSolverName(const std::string& name);
    virtual std::string getSolverName();
    virtual void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg);
    virtual void mazeMatrixCallback(const doogie_msgs::MazeCellMultiArray& matrix_maze);
    virtual ~BaseSolver(){}
    struct ROSParams{
      int& getPlanAttempts();
      float& getRate();
      int plan_attempts{5};
      float rate{0.5};
    };

  protected:
    std::string solver_name_;
    bool start_solver{false};

    ROSParams params_;
    ros::Subscriber doogie_position_sub_;
    ros::Subscriber maze_matrix_sub_;

    ros::NodeHandle nh_;
    ros::NodeHandle ph_{"~"};

    doogie_core::MouseHandle doogie_handle_;
    doogie_core::LocalCell current_cell_;
    doogie_core::MazeMatrixHandle matrix_handle_;
    doogie_msgs::DoogieMoveGoal goal_;
};

} //namespace doogie_core

#endif