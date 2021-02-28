#include <cstddef>
#include <string>
#include "doogie_perception/doogie_perception.hpp"
#include "doogie_core/utils.hpp"
#include "doogie_msgs/DoogiePose.h"

using namespace doogie_core;

namespace doogie_perception {

DoogiePerception::DoogiePerception()
  : is_to_pub_maze_obstacle_matrix_(false) {
  this->loadParameters();
  maze_obstacle_matrix_pub_ = nh_.advertise<doogie_msgs::MazeCellMultiArray>("maze_matrix", 1);
  doogie_pose_sub_ = nh_.subscribe("doogie_pose", 1, &DoogiePerception::doogiePoseCallback, this);
  wall_distances_sub_ = nh_.subscribe("wall_distances", 1, &DoogiePerception::wallDistancesCallback, this);
}

void DoogiePerception::run() {
  ROS_INFO("Doogie perception node has started!");
  maze_obstacle_matrix_pub_.publish(maze_obstacle_matrix_.getMazeMatrix());
  ros::spin();
}

void DoogiePerception::doogiePoseCallback(const doogie_msgs::DoogiePoseConstPtr& doogie_pose) {
  doogie_pose_ = doogie_navigation::MazePose{*doogie_pose};
  is_to_pub_maze_obstacle_matrix_ = true;
}

void DoogiePerception::wallDistancesCallback(const doogie_msgs::WallDistancesConstPtr& wall_distances) {
  if (!is_to_pub_maze_obstacle_matrix_) return;

  doogie_core::LocalCell local_cell = {false, false, false, false};
  uint8_t max_row_val = maze_obstacle_matrix_.getNumberOfRows();
  uint8_t max_column_val = maze_obstacle_matrix_.getNumberOfColumns();
  auto row = doogie_pose_.getRow();
  auto column = doogie_pose_.getColumn();

  if (wall_distances->left_sensor.range < front_dist_threshold_ &&
     wall_distances->right_sensor.range < front_dist_threshold_) local_cell.front_wall = true;
  if (wall_distances->front_left_sensor.range < side_dist_threshold_) local_cell.left_wall = true;
  if (wall_distances->front_right_sensor.range < side_dist_threshold_) {
    local_cell.right_wall = true;
    ROS_INFO_STREAM("PERCEP: RIGH WALL TRUE");
  }

  switch (doogie_pose_.getGlobalOrientation()) {
    case GlobalOrientation::NORTH:
      if (row - 1 < 0) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row - 1, column).north_wall;
      break;
    case GlobalOrientation::SOUTH:
      if (row + 1 > max_row_val) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row + 1, column).south_wall;
      break;
    case GlobalOrientation::EAST:
      if (column - 1 < 0) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row, column - 1).east_wall;
      break;
    case GlobalOrientation::WEST:
      if (column + 1 > max_column_val) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row, column + 1).west_wall;
  }
  const auto& pose= doogie_pose_.toDoogieMsg();
  maze_obstacle_matrix_.setMazeMatrixCell(pose, local_cell, true);
  maze_obstacle_matrix_pub_.publish(maze_obstacle_matrix_.getMazeMatrix());
}

void DoogiePerception::loadParameters() {
  uint8_t number_of_rows;
  uint8_t number_of_columns;
  std::string maze_name;
  std::string log_prefix("doogie_perception");
  ros::NodeHandle ph("~");

  DoogieUtils::getParameterHelper<std::string>(ph, log_prefix, "maze_name", &maze_name, "minus");
  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "front_distance_threshold", &front_dist_threshold_, 0.10);
  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "side_distance_threshold", &side_dist_threshold_, 0.10);

  number_of_rows = DoogieUtils::getNumberOfRows(maze_name);
  number_of_columns = DoogieUtils::getNumberOfColumns(maze_name);
  maze_obstacle_matrix_.initMatrix(number_of_rows, number_of_columns);
}

}  // namespace doogie_perception
