#ifndef DOOGIE_ALGORITHMS_MOUSE_HANDLE_HPP_
#define DOOGIE_ALGORITHMS_MOUSE_HANDLE_HPP_

#include <actionlib/client/simple_action_client.h>
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_msgs/DoogieMoveAction.h"

namespace doogie_algorithms{

class MouseHandle{

public:
MouseHandle();
~MouseHandle();
doogie_msgs::DoogiePosition getPosition();
void setPosition(doogie_msgs::DoogiePosition position);
void move(doogie_msgs::DoogieMoveGoal goal);

private:
doogie_msgs::DoogiePosition position_;
actionlib::SimpleActionClient<doogie_msgs::DoogieMoveAction> move_base_client_;

};
}
#endif // DOOGIE_ALGORITHMS_MOUSE_HANDLE_HPP_