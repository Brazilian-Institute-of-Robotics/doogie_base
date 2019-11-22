#include "doogie_navigation/mouse_handle.hpp"

namespace doogie_navigation{

void MouseHandle::setPosition(doogie_msgs::DoogiePosition position){
  position_ = position;
}

doogie_msgs::DoogiePosition MouseHandle::getPosition(){
  return position_;
}

void MouseHandle::move(Direction){

}

}