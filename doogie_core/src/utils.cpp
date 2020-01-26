#include <ros/ros.h>
#include "doogie_core/utils.hpp"

namespace doogie_core {

uint8_t DoogieUtils::getNumberOfRows(std::string maze_name) {
  return static_cast<uint8_t>(DoogieUtils::getMazeParam<int>(maze_name, "rows", 16));
}

uint8_t DoogieUtils::getNumberOfColumns(std::string maze_name) {
  return static_cast<uint8_t>(DoogieUtils::getMazeParam<int>(maze_name, "columns", 16));
}

float DoogieUtils::getCellSize(std::string maze_name) {
  return DoogieUtils::getMazeParam<float>(maze_name, "cell_size", 0.18);
}

}  // namespace doogie_core
