#include "doogie_core/mouse_handle.hpp"

namespace doogie_core{

MouseHandle::MouseHandle() : move_base_client_("move_base_action_server"){
  position_.row = 0;
  position_.column = 0;
  position_.orientation = 0;

  ROS_INFO("Starting up Move Base Server");
  move_base_client_.waitForServer();
  ROS_INFO("Move Base Server has started. Ready for sending goals...");

}

void MouseHandle::setPosition(doogie_msgs::DoogiePosition position){
  position_ = position;
}

doogie_msgs::DoogiePosition MouseHandle::getPosition(){
  return position_;
}

void MouseHandle::move(doogie_msgs::DoogieMoveGoal goal){
  ros::Rate rate(10);
  move_base_client_.sendGoal(goal);
  while(!move_base_client_.getState().isDone() && ros::ok()){
    ROS_INFO_STREAM("Goal is running.\n State is " + move_base_client_.getState().toString());
    rate.sleep();
  }
}

}