#ifndef DOOGIE_NAVIGATION_LOCAL_CELL_HPP_
#define DOOGIE_NAVIGATION_LOCAL_CELL_HPP_

#include "doogie_msgs/MazeCell.h"
#include "doogie_navigation/maze_pose.hpp"

namespace doogie_navigation {
struct LocalCell {
  LocalCell(doogie::GlobalOrientation orientation, const doogie_msgs::MazeCellPtr& cell);

  bool front_wall_;
  bool back_wall_;
  bool right_wall_;
  bool left_wall_;

  doogie::GlobalOrientation orientation_;
};
}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_LOCAL_CELL_HPP_
