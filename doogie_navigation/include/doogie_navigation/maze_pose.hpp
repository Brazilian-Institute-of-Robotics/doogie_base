#ifndef DOOGIE_NAVIGATION_MAZE_POSE_POSE_HPP_
#define DOOGIE_NAVIGATION_MAZE_POSE_POSE_HPP_

#include <cstdint>

#include <doogie_msgs/DoogieMoveGoal.h>
#include <doogie_msgs/DoogieOrientation.h>
#include <doogie_msgs/DoogiePosition.h>
#include "doogie_msgs/DoogiePose.h"

inline namespace doogie {
  enum Direction : int8_t {
    FRONT = doogie_msgs::DoogieMoveGoal::FRONT,
    BACK = doogie_msgs::DoogieMoveGoal::BACK,
    RIGHT = doogie_msgs::DoogieMoveGoal::RIGHT,
    LEFT = doogie_msgs::DoogieMoveGoal::LEFT
  };

  enum GlobalOrientation {
    NORTH = doogie_msgs::DoogieOrientation::NORTH,
    SOUTH = doogie_msgs::DoogieOrientation::SOUTH,
    EAST = doogie_msgs::DoogieOrientation::EAST,
    WEST = doogie_msgs::DoogieOrientation::WEST
  };
}  // namespace doogie

namespace doogie_navigation {
class MazePose {
 public:
  using Position = doogie_msgs::DoogiePosition;
  using Pose = doogie_msgs::DoogiePose;
  MazePose() = default;

  explicit MazePose(const Pose& doogie_position) {
    position_.row = doogie_position.position.row;
    position_.column = doogie_position.position.column;
  }

  MazePose(int start_row, int start_column, GlobalOrientation start_orientation) {
    position_.row = start_row;
    position_.column = start_column;
    global_orientation_ = start_orientation;
  }

  Pose toDoogieMsg() const {
    Pose msg;
    msg.position.column = getColumn();
    msg.position.row = getRow();
    msg.orientation.direction = getGlobalOrientation();

    return msg;
  }

  void transformOrientation(Direction goal_direction) {
    auto new_orientation = global_orientation_ + goal_direction;
    if (new_orientation > 4) {
      new_orientation -= 4;
    }
    if (new_orientation < 1) {
      new_orientation += 4;
    }
    setGlobalOrientation(GlobalOrientation(new_orientation));
  }

  int getRow() const {
    return position_.row;
  }

  int getColumn() const {
    return position_.column;
  }

  Position getPosition() const {
    return toDoogieMsg().position;
  }

  GlobalOrientation getGlobalOrientation() const {
    return global_orientation_;
  }

  void incrementRow(int rows) {
    position_.row += rows;
  }

  void decrementRow(int rows) {
    position_.row -= rows;
  }

  void incrementColumn(int columns) {
    position_.column += columns;
  }

  void decrementColumn(int columns) {
    position_.column -= columns;
  }

  void setGlobalOrientation(GlobalOrientation orientation) {
    global_orientation_ = orientation;
  }

 private:
  Position position_;
  GlobalOrientation global_orientation_{NORTH};
};

}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MAZE_POSE_POSE_HPP_