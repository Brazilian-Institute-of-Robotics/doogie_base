#include <cstddef>
#include "doogie_navigation/doogie_perception.hpp"

namespace doogie_navigation {

DoogiePerception::DoogiePerception()
  : is_to_pub_maze_obstacle_matrix_(false) {
  this->loadParameters();
  maze_obstacle_matrix_pub_ = nh_.advertise<doogie_msgs::MazeCellMultiArray>("maze_obstacle_matrix", 1);
  doogie_position_sub_ = nh_.subscribe("doogie_position", 1, &DoogiePerception::doogiePositionCallback, this);
  wall_distances_sub_ = nh_.subscribe("wall_distances", 1, &DoogiePerception::wallDistancesCallback, this);
}

void DoogiePerception::run() {
  ROS_INFO("Doogie perception node has started!");
  ros::spin();
}

void DoogiePerception::doogiePositionCallback(const doogie_msgs::DoogiePositionConstPtr& doogie_position) {
  this->doogie_position_ = *doogie_position;
  is_to_pub_maze_obstacle_matrix_ = true;
}

void DoogiePerception::wallDistancesCallback(const doogie_msgs::WallDistancesConstPtr& wall_distances) {
  if (!is_to_pub_maze_obstacle_matrix_) return;

  doogie_algorithms::LocalCell local_cell = {false, false, false, false};
  uint8_t max_row_val = maze_obstacle_matrix_.getNumberOfRows();
  uint8_t max_column_val = maze_obstacle_matrix_.getNumberOfColumns();
  uint8_t row = doogie_position_.row;
  uint8_t column = doogie_position_.column;

  if(wall_distances->left_sensor.range < front_dist_threshold_ &&
     wall_distances->right_sensor.range < front_dist_threshold_) local_cell.front_wall = true;
  if(wall_distances->front_left_sensor.range < side_dist_threshold_) local_cell.left_wall = true;
  if(wall_distances->front_right_sensor.range < side_dist_threshold_) local_cell.right_wall = true;

  switch(doogie_position_.orientation) {
    case doogie_msgs::DoogiePosition::NORTH:
      if(row - 1 < 0) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row - 1, column).north_wall;
      break;
    case doogie_msgs::DoogiePosition::SOUTH:
      if(row + 1 > max_row_val) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row + 1, column).south_wall;
      break;
    case doogie_msgs::DoogiePosition::EAST:
      if(column - 1 < 0) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row, column - 1).east_wall;
      break;
    case doogie_msgs::DoogiePosition::WEST:
      if(column + 1 > max_column_val) {
        local_cell.back_wall = true;
        break;
      }
      local_cell.back_wall = maze_obstacle_matrix_.getMazeMatrixCell(row, column + 1).west_wall;
  }

  maze_obstacle_matrix_.setMazeMatrixCell(doogie_position_, local_cell, true);
  maze_obstacle_matrix_pub_.publish(maze_obstacle_matrix_.getMazeMatrix());
}

void DoogiePerception::loadParameters() {
  int rows_qty;
  int columns_qty;

  if (!nh_.param<int>("maze/rows", rows_qty, 16)) {
    ROS_WARN("No maze/rows parameter found in the parameter server. Using default parameter value: 16");
  }

  if (!nh_.param<int>("maze/columns", columns_qty, 16)) {
    ROS_WARN("No maze/columns parameter found in the parameter server. Using default parameter value: 16");
  }

  if (!nh_.param<float>("maze/front_distance_threshold", front_dist_threshold_, 0.10)) {
    ROS_WARN("No maze/front_distance_threshold parameter found in the parameter server. Using default parameter value: 0.10");
  }

  if (!nh_.param<float>("maze/side_distance_threshold", side_dist_threshold_, 0.10)) {
    ROS_WARN("No maze/side_distance_threshold parameter found in the parameter server. Using default parameter value: 0.10");
  }

  maze_obstacle_matrix_.initMatrix(static_cast<uint8_t>(rows_qty), static_cast<uint8_t>(columns_qty));
}

}
