#ifndef DOOGIE_ALGORITHMS_BASE_SOLVER_HPP_
#define DOOGIE_ALGORITHMS_BASE_SOLVER_HPP_

#include <string>

#include <actionlib/client/action_client.h>

#include "doogie_msgs/DoogieMoveGoal.h"
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_core/mouse_handle.hpp"
#include "doogie_core/maze_matrix_handle.hpp"
namespace doogie_algorithms {

class BaseSolver {
  public:
    BaseSolver();
    BaseSolver(const MazeMatrixPtr maze_matrix);
    doogie_core::MouseHandle& getDoogieHandle();
    virtual void initialize();
    virtual void waitForStart();
    virtual bool isWallFront();
    virtual bool isWallBack();
    virtual bool isWallLeft();
    virtual bool isWallRight();
    virtual bool makePlan() = 0;
    virtual bool move() = 0;
    virtual void sleep();
    virtual void setSolverName(const std::string& name);
    virtual std::string getSolverName();
    virtual void doogiePoseCallback(const doogie_msgs::DoogiePose& position_msg);
    virtual void mazeMatrixCallback(const doogie_msgs::MazeCellMultiArray& matrix_maze);
    virtual ~BaseSolver() = default;

  protected:
    struct ROSParams {
      float rate{0.5};
    };

    virtual void loadParams(ROSParams& params);
    ros::NodeHandle& getNodeHandle();
    ros::NodeHandle& getPrivateNodeHandle();

    ROSParams params_;
  
    ros::Subscriber doogie_pose_sub_;
    ros::Subscriber maze_matrix_sub_;

    doogie_msgs::DoogieMoveGoal goal_;

  private:
    bool start_solver{false};
    std::string solver_name_;

    ros::NodeHandle nh_;
    ros::NodeHandle ph_{"~"};

    doogie_core::MazeMatrixHandle matrix_handle_;
    doogie_core::MouseHandle doogie_handle_;
    
    doogie_core::LocalCell current_cell_;
};

}  // namespace doogie_algorithms

#endif  // DOOGIE_ALGORITHMS_BASE_SOLVER_HPP_