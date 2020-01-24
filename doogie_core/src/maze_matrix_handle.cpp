#include <std_msgs/MultiArrayDimension.h>
#include "doogie_core/maze_matrix_handle.hpp"

namespace doogie_core {

MazeMatrixHandle::MazeMatrixHandle() {}

MazeMatrixHandle::MazeMatrixHandle(uint8_t row_qty, uint8_t column_qty) {
  this->initMatrix(row_qty, column_qty);
}

void MazeMatrixHandle::initMatrix(uint8_t row_qty, uint8_t column_qty) {
  maze_matrix_.layout.dim.push_back(std_msgs::MultiArrayDimension());
  maze_matrix_.layout.dim.push_back(std_msgs::MultiArrayDimension());

  maze_matrix_.layout.dim[0].size = row_qty;
  maze_matrix_.layout.dim[0].label = "row";
  maze_matrix_.layout.dim[0].stride = row_qty * column_qty;
  maze_matrix_.layout.dim[1].size = column_qty;
  maze_matrix_.layout.dim[1].label = "column";
  maze_matrix_.layout.dim[1].stride = column_qty;

  for (std::size_t i = 0; i < row_qty * column_qty; i++) {
    maze_matrix_.data.push_back(doogie_msgs::MazeCell());
  }


  for (std::size_t i = 0; i < row_qty; i++) {
    for (std::size_t j = 0; j < column_qty; j++) {
      doogie_msgs::MazeCell maze_cell;
      if (i == 0) maze_cell.south_wall = true;
      if (i == row_qty - 1) maze_cell.north_wall = true;
      if (j == 0) maze_cell.west_wall = true;
      if (j == column_qty - 1) maze_cell.east_wall = true;
      this->setMazeMatrixCell(i, j, maze_cell);
    }
  }
}

doogie_msgs::MazeCell MazeMatrixHandle::getMazeMatrixCell(MazeMatrix maze_matrix, uint8_t row, uint8_t column) {
  return maze_matrix.data[row + maze_matrix.layout.dim[1].stride * column];
}

doogie_msgs::MazeCell MazeMatrixHandle::getMazeMatrixCell(uint8_t row, uint8_t column) {
  return this->maze_matrix_.data[row + this->maze_matrix_.layout.dim[1].stride * column];
}

MazeMatrix MazeMatrixHandle::getMazeMatrix() {
  return this->maze_matrix_;
}

void MazeMatrixHandle::setMazeMatrixCell(uint8_t row, uint8_t column, doogie_msgs::MazeCell maze_cell_walls) {
  this->maze_matrix_.data[row + this->maze_matrix_.layout.dim[1].stride * column] = maze_cell_walls;
}

void MazeMatrixHandle::setMazeMatrixCell(doogie_msgs::DoogiePosition doogie_position, LocalCell local_cell_walls,
                                         bool visited) {
  doogie_msgs::MazeCell maze_cell_walls = this->localToGlobalCell(doogie_position.orientation, local_cell_walls);
  maze_cell_walls.visited = visited;
  this->setMazeMatrixCell(doogie_position.row, doogie_position.column, maze_cell_walls);
}

void MazeMatrixHandle::setMazeMatrixCell(uint8_t row, uint8_t column, bool north_wall, bool south_wall, bool east_wall,
                                         bool west_wall, bool visited) {
  doogie_msgs::MazeCell maze_cell_walls;
  maze_cell_walls.north_wall = north_wall;
  maze_cell_walls.south_wall = south_wall;
  maze_cell_walls.east_wall = east_wall;
  maze_cell_walls.west_wall = west_wall;

  this->setMazeMatrixCell(row, column, maze_cell_walls);
}

LocalCell MazeMatrixHandle::globalToLocal(uint8_t orientation, doogie_msgs::MazeCell maze_cell) {
  LocalCell local_cell;

  switch (orientation){
    case doogie_msgs::DoogiePosition::NORTH:
      local_cell.front_wall = maze_cell.north_wall;
      local_cell.back_wall = maze_cell.south_wall;
      local_cell.right_wall = maze_cell.east_wall;
      local_cell.left_wall = maze_cell.west_wall;
      return local_cell;
    case doogie_msgs::DoogiePosition::SOUTH:
      local_cell.front_wall = maze_cell.south_wall;
      local_cell.back_wall = maze_cell.north_wall;
      local_cell.right_wall = maze_cell.west_wall;
      local_cell.left_wall = maze_cell.east_wall;
      return local_cell;
    case doogie_msgs::DoogiePosition::EAST:
      local_cell.front_wall = maze_cell.east_wall;
      local_cell.back_wall = maze_cell.west_wall;
      local_cell.right_wall = maze_cell.south_wall;
      local_cell.left_wall = maze_cell.north_wall;
      return local_cell;
    case doogie_msgs::DoogiePosition::WEST:
      local_cell.front_wall = maze_cell.west_wall;
      local_cell.back_wall = maze_cell.east_wall;
      local_cell.right_wall = maze_cell.north_wall;
      local_cell.left_wall = maze_cell.south_wall;
      return local_cell;
  }
}

LocalCell MazeMatrixHandle::globalToLocalCell(MazeMatrix maze_matrix, doogie_msgs::DoogiePosition doogie_position) {
  doogie_msgs::MazeCell maze_cell = MazeMatrixHandle::getMazeMatrixCell(maze_matrix, doogie_position.row,
                                                                        doogie_position.column);
  LocalCell local_cell = MazeMatrixHandle::globalToLocal(doogie_position.orientation, maze_cell);
  return local_cell;
}

LocalCell MazeMatrixHandle::globalToLocalCell(doogie_msgs::DoogiePosition doogie_position) {
  doogie_msgs::MazeCell maze_cell = this->getMazeMatrixCell(doogie_position.row, doogie_position.column);
  LocalCell local_cell = MazeMatrixHandle::globalToLocal(doogie_position.orientation, maze_cell);
  return local_cell;
}

doogie_msgs::MazeCell MazeMatrixHandle::localToGlobalCell(uint8_t orientation, LocalCell local_cell) {
  doogie_msgs::MazeCell maze_cell;
  
  switch (orientation){
    case doogie_msgs::DoogiePosition::NORTH:
      maze_cell.north_wall = local_cell.front_wall;
      maze_cell.south_wall = local_cell.back_wall;
      maze_cell.east_wall = local_cell.right_wall;
      maze_cell.west_wall = local_cell.left_wall;
      return maze_cell;
    case doogie_msgs::DoogiePosition::SOUTH:
      maze_cell.north_wall = local_cell.back_wall;
      maze_cell.south_wall = local_cell.front_wall;
      maze_cell.east_wall = local_cell.left_wall;
      maze_cell.west_wall = local_cell.right_wall;
      return maze_cell;
    case doogie_msgs::DoogiePosition::EAST:
      maze_cell.north_wall = local_cell.left_wall;
      maze_cell.south_wall = local_cell.right_wall;
      maze_cell.east_wall = local_cell.front_wall;
      maze_cell.west_wall = local_cell.back_wall;
      return maze_cell;
    case doogie_msgs::DoogiePosition::WEST:
      maze_cell.north_wall = local_cell.right_wall;
      maze_cell.south_wall = local_cell.left_wall;
      maze_cell.east_wall = local_cell.back_wall;
      maze_cell.west_wall = local_cell.front_wall;
      return maze_cell;
  }
}

uint8_t MazeMatrixHandle::getNumberOfRows() {
  return this->maze_matrix_.layout.dim[0].size;
}

uint8_t MazeMatrixHandle::getNumberOfColumns() {
  return this->maze_matrix_.layout.dim[1].size;
}

}
