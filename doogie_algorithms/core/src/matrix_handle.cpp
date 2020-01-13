#include <std_msgs/MultiArrayDimension.h>
#include "doogie_algorithms/matrix_handle.hpp"

namespace doogie_algorithms {

MatrixHandle::MatrixHandle() {
  mat.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mat.layout.dim.push_back(std_msgs::MultiArrayDimension());
}

void MatrixHandle::initMatrix(int i_size, int j_size, std::string i_label, std::string j_label, int fill, int offset) {
  mat.layout.dim[0].size = i_size;
  mat.layout.dim[1].size = j_size;
  mat.layout.dim[0].label = i_label;
  mat.layout.dim[1]. label = j_label;
  mat.layout.dim[0].stride = i_size*j_size;
  mat.layout.dim[1].stride = j_size;
  mat.layout.data_offset = offset;
  doogie_msgs::MazeCellMultiArray empty_list;

  for (std::size_t i = 0; i < i_size; i++) {
      for (std::size_t j = 0; j < j_size; j++) {
          mat.data.push_back(doogie_msgs::MazeCell());
      }
  }
}

doogie_msgs::MazeCell MatrixHandle::getMatrixElement(doogie_msgs::MazeCellMultiArray matrix, int i, int j) {
    int dstride1 = matrix.layout.dim[1].stride;
    int offset = matrix.layout.data_offset;
    return matrix.data[offset + i + dstride1*j];
}

void MatrixHandle::setMatrixElement(doogie_msgs::MazeCellMultiArray matrix, int i, int j, bool north_element,
                                    bool south_element, bool east_element, bool west_element, bool visited) {
  int dstride1 = matrix.layout.dim[1].stride;
  int offset = matrix.layout.data_offset;
  matrix.data[offset + i + dstride1*j].north_wall = north_element;
  matrix.data[offset + i + dstride1*j].south_wall = south_element;
  matrix.data[offset + i + dstride1*j].east_wall = east_element;
  matrix.data[offset + i + dstride1*j].west_wall = west_element;
  matrix.data[offset + i + dstride1*j].visited = visited;
}

LocalCell MatrixHandle::GlobalToLocal(doogie_msgs::DoogiePosition doogie_position){
  LocalCell local_cell;
  doogie_msgs::MazeCell global_cell = getMatrixElement(mat, doogie_position.row, doogie_position.column);
  switch (doogie_position.orientation){
    case 1:
      local_cell.front_wall = global_cell.north_wall;
      local_cell.back_wall = global_cell.south_wall;
      local_cell.right_wall = global_cell.east_wall;
      local_cell.left_wall = global_cell.west_wall;
      break;
    
    case 2:
      local_cell.front_wall = global_cell.east_wall;
      local_cell.back_wall = global_cell.west_wall;
      local_cell.right_wall = global_cell.south_wall;
      local_cell.left_wall = global_cell.north_wall;
      break;
    
    case 3:
      local_cell.front_wall = global_cell.south_wall;
      local_cell.back_wall = global_cell.north_wall;
      local_cell.right_wall = global_cell.west_wall;
      local_cell.left_wall = global_cell.east_wall;
      break;

    case 4:
      local_cell.front_wall = global_cell.west_wall;
      local_cell.back_wall = global_cell.east_wall;
      local_cell.right_wall = global_cell.north_wall;
      local_cell.left_wall = global_cell.south_wall;
      break;
  }

}

}
