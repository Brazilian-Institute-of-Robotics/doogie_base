#include "doogie_navigation/move_base_params.hpp"

namespace doogie_navigation {
  MoveBaseParams::MoveBaseParams(const ros::NodeHandle& robot_nh) {
    distance_tolerance = loadParam(robot_nh, "distance_tolerance");
    angle_tolerance = loadParam(robot_nh, "angle_tolerance");
    linear_velocity = loadParam(robot_nh, "linear_velocity");
    angular_velocity = loadParam(robot_nh,"angular_velocity");
    loop_frequency = loadParam(robot_nh, "loop_frequency");
    cell_size = loadParam(robot_nh, "cell_size");
    row_init_position = loadParam(robot_nh, "row_init_position");
    column_init_position = loadParam(robot_nh, "column_init_position");
  }

  double MoveBaseParams::loadParam(const ros::NodeHandle& private_nh, const std::string& param_name) {
    if(!private_nh.hasParam(param_name)) {
      ROS_ERROR_STREAM("No parameter with name:" << param_name << " was found in the parameter server.");
      throw std::runtime_error("");
    }
    double param_value{};
    private_nh.getParam(param_name, param_value);
    return param_value;
  }
}  // namespace doogie_navigation