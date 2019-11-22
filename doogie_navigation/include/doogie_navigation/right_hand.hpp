#include <ros/ros.h>
#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_navigation/matrix_handle.hpp"
#include "doogie_navigation/mouse_handle.hpp"

namespace doogie_navigation{

class RightHandStrategy{
  public:
  void init();
  void mazeWallsCallback(const doogie_msgs::MazeCellMultiArray& matrix_msg);
  void doogiePositionCallback(const doogie_msgs::DoogiePosition& position_msg);
  private:
  MouseHandle doogie_handle_;
  LocalCell current_cell_;
  doogie_msgs::MazeCellMultiArrayPtr matrix_maze_;
  MatrixHandle matrix_handle_;
  ros::NodeHandle nh_;
  ros::Subscriber maze_walls_sub_;
  ros::Subscriber doogie_position_sub_;
};

}