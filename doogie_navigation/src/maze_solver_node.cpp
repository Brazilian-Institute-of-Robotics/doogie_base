#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "doogie_core/solver_plugin_loader.hpp"


const std::string& BASE_CLASS_PACKAGE = "doogie_core";
const std::string& BASE_CLASS = BASE_CLASS_PACKAGE+"::BaseSolver";

void loop(const boost::shared_ptr<doogie_core::BaseSolver>& solver);

int main (int argc, char** argv)
{
  std::string plugin_param_name;
  boost::shared_ptr<doogie_core::BaseSolver> solver;
  
  ros::init(argc, argv, "maze_solver");
  ros::NodeHandle ph("~");
  
  doogie_core::SolverPluginLoader plugin_loader(BASE_CLASS_PACKAGE, BASE_CLASS);

  if (ph.getParam("/solver_plugin", plugin_param_name))
  {  
    solver = plugin_loader.getSolverInstance(plugin_param_name);
    loop(solver);
    solver.reset();
  }
  
  else{
    ROS_ERROR("Could not find /solver_plugin param, check if it's setted");
  }
    
}

void loop(const boost::shared_ptr<doogie_core::BaseSolver>& solver){
  while(ros::ok()){
    solver->makePlan();
    solver->move();
    solver->sleep();
  }
}