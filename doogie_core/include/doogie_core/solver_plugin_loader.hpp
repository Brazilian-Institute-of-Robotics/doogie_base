#include "doogie_core/base_solver.hpp"
#include <pluginlib/class_loader.h>

namespace doogie_core{

class SolverPluginLoader{
  public:
    SolverPluginLoader(const std::string& package, const std::string& base_class) : 
                                          solver_loader_(package, base_class){}
    boost::shared_ptr<doogie_core::BaseSolver> getSolverInstance(const std::string& solver_param_name);

  private:
    std::string solver_plugin_name_;
    pluginlib::ClassLoader<doogie_core::BaseSolver> solver_loader_;
    boost::shared_ptr<doogie_core::BaseSolver> solver_;
};

};
