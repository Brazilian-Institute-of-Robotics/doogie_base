#ifndef DOOGIE_NAVIGATION_MOVE_BASE_PARAMS_HPP_
#define DOOGIE_NAVIGATION_MOVE_BASE_PARAMS_HPP_

#include <ros/ros.h>

namespace doogie_navigation {
class MoveBaseParams {
 public:
  MoveBaseParams(const ros::NodeHandle& axis_nh);
  double loadParam(const ros::NodeHandle& private_nh, const std::string& param_name);

  double distance_tolerance;
  double angle_tolerance;
  double linear_velocity;
  double angular_velocity;
  double loop_frequency;
  double cell_size;
  double row_init_position;
  double column_init_position;

};
}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_BASE_PARAMS_HPP_
