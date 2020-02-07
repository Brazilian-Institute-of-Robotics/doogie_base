#ifndef DOOGIE_CORE_UTILS_HPP_
#define DOOGIE_CORE_UTILS_HPP_

#include <cstddef>
#include <string>
#include <ros/ros.h>

namespace doogie_core {

/**
 * @brief This class join all commons methods used in the other applications.
 * 
 * The methods that are common to more one application, e.g get the number of
 * rows of the maze, will be placed here. This approach avoid mistakes when the
 * same task is made in different codes. 
 */
class DoogieUtils {
 public:
  /**
   * @brief Helper method to get parameters from parameter server.
   * 
   * This method will checks if a param with @p param_name exists in parameter
   * server. If it isn't exists, the @p default_value will be assigned to @p param.
   * 
   * @tparam T Data type of the parameter.
   * @param nh ROS Nodehandle.
   * @param param_name Name of the parameter.
   * @param param Pointer to variable which will store the parameter value.
   * @param default_value Value assigned to @p param if the parameter was not found.
   */
  template <typename T>
  static void getParameterHelper(const ros::NodeHandle &nh, std::string log_prefix,
                                std::string param_name, T *param, T default_value) {
    if (!nh.param<T>(param_name, *param, default_value)) {
      ROS_WARN_STREAM_NAMED(log_prefix, "No " << param_name <<
      " parameter found in the parameter server. Using default parameter value: " << default_value);
    }
  }

  /**
   * @brief Get the number of rows in the maze specified by @p maze_mame.
   * 
   * @param maze_name The name of the maze used.
   * @return uint8_t Number of rows.
   */
  static uint8_t getNumberOfRows(std::string maze_name);
  
  /**
   * @brief Get the number of columns in the maze specified by @p maze_mame.
   * 
   * @param maze_name The name of the maze used.
   * @return uint8_t Number of columns.
   */
  static uint8_t getNumberOfColumns(std::string maze_name);

  /**
   * @brief Get the cell size of the maze specified by @p maze_mame.
   * 
   * @param maze_name The name of the maze used.
   * @return float Cell size in meters.
   */
  static float getCellSize(std::string maze_name);

 private:
  /**
   * @brief Get the global maze param.
   * 
   * @tparam T Data type of the parameter.
   * @param maze_name The name of the maze used.
   * @param param_name Name of the parameter.
   * @param default_value Value assigned to @p param if the parameter was not found.
   * @return T The value of the parameter.
   */
  template <typename T>
  static T getMazeParam(std::string maze_name, std::string param_name, T default_value) {
    ros::NodeHandle nh;
    T param_value; 

    DoogieUtils::getParameterHelper<T>(nh, "doogie_utils", maze_name + "/" + param_name, &param_value, default_value);

    return param_value;
  }
};

} // namespace doogie_core

#endif  // DOOGIE_CORE_UTILS_HPP_
