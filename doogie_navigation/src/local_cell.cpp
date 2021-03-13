#include "doogie_navigation/local_cell.hpp"

namespace doogie_navigation {
LocalCell::LocalCell(doogie::GlobalOrientation orientation, const doogie_msgs::MazeCellPtr& cell) {
  orientation_ = orientation;

  if (orientation_ == doogie::GlobalOrientation::NORTH) {
    front_wall_ = cell->north_wall;
    back_wall_ = cell->south_wall;
    right_wall_ = cell->east_wall;
    left_wall_ = cell->west_wall;
  }

  if (orientation_ == doogie::GlobalOrientation::SOUTH) {
    front_wall_ = cell->south_wall;
    back_wall_ = cell->north_wall;
    right_wall_ = cell->west_wall;
    left_wall_ = cell->east_wall;
  }

  if (orientation_ == doogie::GlobalOrientation::EAST) {
    front_wall_ = cell->east_wall;
    back_wall_ = cell->west_wall;
    right_wall_ = cell->south_wall;
    left_wall_ = cell->north_wall;
  }

  if (orientation_ == doogie::GlobalOrientation::WEST) {
    front_wall_ = cell->west_wall;
    back_wall_ = cell->east_wall;
    right_wall_ = cell->north_wall;
    left_wall_ = cell->south_wall;
  }
}
}  // namespace doogie_navigation
