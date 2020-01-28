#include "doogie_core/base_solver.hpp"
#include <pluginlib/class_loader.h>

namespace doogie_core{

class SolverPluginLoader{
  public:
    SolverPluginLoader(const std::string& package, const std::string& base_class);
    void setSolverNameFromParams();
    std::string getSolverName();
    boost::shared_ptr<doogie_core::BaseSolver> getSolverInstance();                                          
    boost::shared_ptr<doogie_core::BaseSolver> getSolverInstance(const std::string& solver_param_name);

  private:
    ros::NodeHandle ph_{"~"};
    std::string solver_name_{"doogie_algorithms/RightHandSolverPlugin"};
    pluginlib::ClassLoader<doogie_core::BaseSolver> solver_loader_;
    boost::shared_ptr<doogie_core::BaseSolver> solver_;
};

} //namespace doogie_core