#ifndef DOOGIE_CORE_MOUSE_HANDLE_HPP_
#define DOOGIE_CORE_MOUSE_HANDLE_HPP_

#include <actionlib/client/simple_action_client.h>
#include "doogie_msgs/DoogiePose.h"
#include "doogie_msgs/DoogieMoveAction.h"

namespace doogie_core {

class MouseHandle {

public:
MouseHandle();
doogie_msgs::DoogiePose getPose();
void setPose(doogie_msgs::DoogiePose pose);
void move(doogie_msgs::DoogieMoveGoal goal);

private:
doogie_msgs::DoogiePose pose_;
actionlib::SimpleActionClient<doogie_msgs::DoogieMoveAction> move_base_client_;

};
}
#endif // DOOGIE_CORE_MOUSE_HANDLE_HPP_