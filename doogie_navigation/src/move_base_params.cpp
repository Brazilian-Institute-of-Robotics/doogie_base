#include "doogie_navigation/move_base_params.hpp"

namespace doogie_navigation {
  MoveBaseParams::MoveBaseParams(const ros::NodeHandle& robot_nh) {
    ros::NodeHandle private_nh{"~"};
    localization_plugin = loadParam<std::string>(private_nh, "localization_plugin");
    distance_tolerance = loadParam<double>(private_nh, "distance_tolerance");
    angle_tolerance = loadParam<double>(private_nh, "angle_tolerance");
    linear_velocity = loadParam<double>(private_nh, "linear_velocity");
    angular_velocity = loadParam<double>(private_nh,"angular_velocity");
    loop_frequency = loadParam<double>(private_nh, "loop_frequency");
    cell_size = loadParam<double>(private_nh, "cell_size");
    row_init_position = loadParam<int>(private_nh, "row_init_position");
    column_init_position = loadParam<int>(private_nh, "column_init_position");
  }

  template <typename T>
  T MoveBaseParams::loadParam(const ros::NodeHandle& private_nh, const std::string& param_name) {
    if(!private_nh.hasParam(param_name)) {
      ROS_ERROR_STREAM("No parameter with name: \"" << param_name << "\" was found in the parameter server.");
      throw std::runtime_error("");
    }
    T param_value{};
    private_nh.getParam(param_name, param_value);
    return param_value;
  }

  template double MoveBaseParams::loadParam<double>(const ros::NodeHandle& private_nh, const std::string& param_name);
  template int MoveBaseParams::loadParam<int>(const ros::NodeHandle& private_nh, const std::string& param_name);
}  // namespace doogie_navigation