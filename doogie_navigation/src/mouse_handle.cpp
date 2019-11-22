#include "doogie_navigation/mouse_handle.hpp"

namespace doogie_navigation{

MouseHandle::MouseHandle() : move_base_client_("move_base", true){
  ROS_INFO("Waiting for move_base server to start.");
  move_base_client_.waitForServer();
  ROS_INFO("move_base server started, sending goal.");
}

void MouseHandle::setPosition(doogie_msgs::DoogiePosition position){
  position_ = position;
}

doogie_msgs::DoogiePosition MouseHandle::getPosition(){
  return position_;
}

void MouseHandle::move(Direction direction){
  goal_handle_.direction = direction;
  move_base_client_.sendGoal(goal_handle_);
}

}