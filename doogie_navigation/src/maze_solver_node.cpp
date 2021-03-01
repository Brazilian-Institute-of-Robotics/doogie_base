#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "doogie_algorithms/solver_plugin_loader.hpp"
#include "doogie_algorithms/base_solver.hpp"


const std::string& BASE_CLASS_PACKAGE = "doogie_algorithms";
const std::string& BASE_CLASS = BASE_CLASS_PACKAGE+"::BaseSolver";

void loop(const boost::shared_ptr<doogie_algorithms::BaseSolver>& solver);

int main (int argc, char** argv){
  
  ros::init(argc, argv, "maze_solver");
  ros::NodeHandle ph("~");
  
  doogie_algorithms::SolverPluginLoader plugin_loader(BASE_CLASS_PACKAGE, BASE_CLASS);
  boost::shared_ptr<doogie_algorithms::BaseSolver> solver = plugin_loader.getSolverInstance();

  solver->waitForStart();
  loop(solver);
  solver.reset();
}

void loop(const boost::shared_ptr<doogie_algorithms::BaseSolver>& solver){
  while(ros::ok()){
    solver->makePlan();
    solver->move();
    solver->sleep();
  }
}