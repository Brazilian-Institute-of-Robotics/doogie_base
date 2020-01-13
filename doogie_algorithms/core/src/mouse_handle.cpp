#include "doogie_algorithms/mouse_handle.hpp"

namespace doogie_algorithms{

MouseHandle::MouseHandle() : move_base_client_("move_base_action_server"){
  ROS_INFO("Starting up Move Base Server");
  // move_base_client_.waitForServer();
  ROS_INFO("Move Base Server has started. Ready for sending goals...");

}

MouseHandle::~MouseHandle(){
  
}

void MouseHandle::setPosition(doogie_msgs::DoogiePosition position){
  position_ = position;
}

doogie_msgs::DoogiePosition MouseHandle::getPosition(){
  return position_;
}

void MouseHandle::move(doogie_msgs::DoogieMoveGoal goal){
  move_base_client_.sendGoal(goal);
}

}