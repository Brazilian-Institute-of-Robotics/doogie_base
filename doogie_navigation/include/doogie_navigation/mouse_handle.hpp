#ifndef DOOGIE_NAVIGATION_MOUSE_HANDLE
#define DOOGIE_NAVIGATION_MOUSE_HANDLE

#include "doogie_msgs/DoogiePosition.h"

namespace doogie_navigation{
enum Direction{
FWD, BACK, RIGHT, LEFT
};

class MouseHandle{

public:
doogie_msgs::DoogiePosition getPosition();
void setPosition(doogie_msgs::DoogiePosition position);
void move(Direction);

private:
doogie_msgs::DoogiePosition position_;

};
}
#endif 