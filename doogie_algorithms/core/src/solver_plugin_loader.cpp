#include "doogie_algorithms/solver_plugin_loader.hpp"

using namespace doogie_algorithms;
boost::shared_ptr<doogie_algorithms::BaseSolver> SolverPluginLoader::getSolverInstance(const std::string& solver_param_name){
  try
  {
    solver_ = solver_loader_.createInstance(solver_param_name);
    return solver_; 
  }
    catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL("The %s plugin failed to load.", solver_param_name.c_str());
    throw pluginlib::PluginlibException (ex.what());
  }

}