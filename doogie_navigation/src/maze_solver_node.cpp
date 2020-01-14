#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "doogie_algorithms/solver_plugin_loader.hpp"

int main (int argc, char** argv)
{
  const std::string& package = "doogie_algorithms";
  const std::string& base_class = "doogie_algorithms::BaseSolver";
  std::string plugin_param_name;
  boost::shared_ptr<doogie_algorithms::BaseSolver> solver;
  
  ros::init(argc, argv, "maze_solver");
  ros::NodeHandle nh;
  
  doogie_algorithms::SolverPluginLoader plugin_loader("doogie_algorithms", "doogie_algorithms::BaseSolver");

  if (nh.getParam("/solver_plugin", plugin_param_name))
  {  
    solver = plugin_loader.getSolverInstance(plugin_param_name);
    solver->init();
    solver.reset();
  }
  
  else{
    ROS_ERROR("Could not find /solver_plugin param, check if it's setted");
  }
    
}