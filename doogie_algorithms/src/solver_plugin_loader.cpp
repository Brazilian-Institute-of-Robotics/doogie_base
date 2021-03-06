#include "doogie_algorithms/solver_plugin_loader.hpp"

namespace doogie_algorithms {

SolverPluginLoader::SolverPluginLoader(const std::string& package, const std::string& base_class)
: solver_loader_(package, base_class){}

std::string SolverPluginLoader::getSolverName(){
  return solver_name_;
}

void SolverPluginLoader::setSolverNameFromParams(){
  if (!ph_.getParam("/solver_plugin", solver_name_)){
    ROS_INFO_STREAM("Using default solver: " << solver_name_);
  }
}

boost::shared_ptr<doogie_algorithms::BaseSolver> SolverPluginLoader::getSolverInstance(){
  setSolverNameFromParams();
  getSolverInstance(solver_name_);
  return solver_;
}

boost::shared_ptr<doogie_algorithms::BaseSolver> SolverPluginLoader::getSolverInstance(const std::string& solver_name){
  try
  {
    solver_ = solver_loader_.createInstance(solver_name);
    solver_->setSolverName(solver_name_);
    solver_->initialize();
    return solver_; 
  }
    catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL("The %s plugin failed to load.", solver_name_.c_str());
    throw pluginlib::PluginlibException (ex.what());
  }
}

}  // namespace doogie_algorithms