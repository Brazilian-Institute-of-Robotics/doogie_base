#ifndef DOOGIE_NAVIGATION_MOVE_BASE_PARAMS_HPP_
#define DOOGIE_NAVIGATION_MOVE_BASE_PARAMS_HPP_

#include <ros/ros.h>

namespace doogie_navigation {
class MoveBaseParams {
 public:
  MoveBaseParams(const ros::NodeHandle& axis_nh);
  template <typename T>
  T loadParam(const ros::NodeHandle& private_nh, const std::string& param_name);

  std::string localization_plugin{"doogie_localization/OdomController"};
  double distance_tolerance;
  double angle_tolerance;
  double linear_velocity;
  double angular_velocity;
  double loop_frequency;
  double cell_size;
  int row_init_position;
  int column_init_position;

};
}  // namespace doogie_navigation

#endif  // DOOGIE_NAVIGATION_MOVE_BASE_PARAMS_HPP_
