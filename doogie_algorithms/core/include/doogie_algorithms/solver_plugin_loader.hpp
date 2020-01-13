#include "doogie_algorithms/base_solver.hpp"
#include <pluginlib/class_loader.h>

namespace doogie_algorithms{

class SolverPluginLoader{
  public:
    SolverPluginLoader(const std::string& package, const std::string& base_class) : 
                                          solver_loader_(package, base_class){}
    boost::shared_ptr<doogie_algorithms::BaseSolver> getSolverInstance(const std::string& solver_param_name);

  private:
    std::string solver_plugin_name_;
    pluginlib::ClassLoader<doogie_algorithms::BaseSolver> solver_loader_;
    boost::shared_ptr<doogie_algorithms::BaseSolver> solver_;
};

};
