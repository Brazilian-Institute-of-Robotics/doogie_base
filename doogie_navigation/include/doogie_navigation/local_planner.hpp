#ifndef DOOGIE_NAVIGATION_LOCAL_PLANNER_HPP_
#define DOOGIE_NAVIGATION_LOCAL_PLANNER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "doogie_msgs/WallDistances.h"

namespace doogie_navigation{

class LocalPlanner{
    public:
    LocalPlanner();
    void ir_sensors_callback(const doogie_msgs::WallDistancesConstPtr& ir_sensors_msg);

    private:
    ros::NodeHandle nh;
    float threshold_{0.5}; // in cm
    geometry_msgs::Twist twist_cmd_;
    ros::Subscriber ir_sensors_sub;
};

} //namespace doogie_navigation

#endif