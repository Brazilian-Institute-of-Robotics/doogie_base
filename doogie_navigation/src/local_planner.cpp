#include "doogie_navigation/local_planner.hpp"

namespace doogie_navigation{
    LocalPlanner::LocalPlanner(){
        ir_sensors_sub = nh.subscribe("wall_distances", 1, &LocalPlanner::ir_sensors_callback, this);
    }
    void LocalPlanner::ir_sensors_callback(const doogie_msgs::WallDistancesConstPtr& ir_sensors_msg){
        if( ir_sensors_msg->right_sensor.range <= threshold_ ){
            twist_cmd_.angular.z = 0.01 * (ir_sensors_msg->right_sensor.range - threshold_);
        }
        if(ir_sensors_msg->left_sensor.range <= threshold_){
            twist_cmd_.angular.z = -0.01 * (ir_sensors_msg->right_sensor.range - threshold_);
        }
    }

}