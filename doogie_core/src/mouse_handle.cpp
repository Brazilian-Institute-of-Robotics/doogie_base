#include "doogie_core/mouse_handle.hpp"

namespace doogie_core{

MouseHandle::MouseHandle() : move_base_client_("move_base_action_server"){
  pose_.position.row = 0;
  pose_.position.column = 0;
  pose_.orientation.direction = 0;

  ROS_INFO("Starting up Move Base Server");
  move_base_client_.waitForServer();
  ROS_INFO("Move Base Server has started. Ready for sending goals...");

}

void MouseHandle::setPose(doogie_msgs::DoogiePose pose){
  pose_ = pose;
}

doogie_msgs::DoogiePose MouseHandle::getPose(){
  return pose_;
}

void MouseHandle::move(doogie_msgs::DoogieMoveGoal goal){
  ros::Rate rate(10);
  move_base_client_.sendGoal(goal);
  while(!move_base_client_.getState().isDone() && ros::ok()){
    ROS_DEBUG_STREAM_THROTTLE(0.5,"Goal is running.\n State is " + move_base_client_.getState().toString());
    rate.sleep();
  }
}

}