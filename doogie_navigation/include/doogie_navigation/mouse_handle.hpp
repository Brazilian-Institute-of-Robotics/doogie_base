#ifndef DOOGIE_NAVIGATION_MOUSE_HANDLE
#define DOOGIE_NAVIGATION_MOUSE_HANDLE

#include <actionlib/client/simple_action_client.h>
#include "doogie_msgs/DoogiePosition.h"
#include "doogie_msgs/DoogieMoveGoal.h"
#include "doogie_msgs/DoogieMoveAction.h"

namespace doogie_navigation{
enum Direction{
FWD=1u, BACK, RIGHT, LEFT
};

class MouseHandle{

public:
MouseHandle();
doogie_msgs::DoogiePosition getPosition();
void setPosition(doogie_msgs::DoogiePosition position);
void move(Direction direction);

private:
doogie_msgs::DoogiePosition position_;
actionlib::SimpleActionClient<doogie_msgs::DoogieMoveAction> move_base_client_;
doogie_msgs::DoogieMoveGoal goal_handle_;


};
}
#endif 