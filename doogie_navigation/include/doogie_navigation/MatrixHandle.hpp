#ifndef DOOGIE_NAVIGATION_WALL_DETECTOR_HPP_
#define DOOGIE_NAVIGATION_WALL_DETECTOR_HPP_

#include <ros/ros.h>
#include "doogie_msgs/MazeCell.h"
#include "doogie_msgs/MazeCellMultiArray.h"

#include <string>


namespace doogie_navigation{

class MatrixHandle{
  public:
  MatrixHandle();
  void initMatrix(int i_size, int j_size, std::string i_label, std::string j_label, int fill=0, int offset=0);
  doogie_msgs::MazeCell getMatrixElement(doogie_msgs::MazeCellMultiArray matrix, int i, int j);
  void setMatrixElement(doogie_msgs::MazeCellMultiArray matrix, int i, int j,bool northElement, 
                        bool southElement, bool eastElement, bool westElement, bool visited);
  
  private:
  doogie_msgs::MazeCellMultiArray mat;
};

}

#endif  