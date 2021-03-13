#ifndef DOOGIE_ALGORITHMS_SOLVER_PLUGIN_LOADER_HPP_
#define DOOGIE_ALGORITHMS_SOLVER_PLUGIN_LOADER_HPP_

#include <pluginlib/class_loader.h>
#include "doogie_algorithms/base_solver.hpp"

namespace doogie_algorithms {

class SolverPluginLoader {
  public:
    SolverPluginLoader(const std::string& package, const std::string& base_class);
    void setSolverNameFromParams();
    std::string getSolverName();
    boost::shared_ptr<doogie_algorithms::BaseSolver> getSolverInstance();                                          
    boost::shared_ptr<doogie_algorithms::BaseSolver> getSolverInstance(const std::string& solver_param_name);

  private:
    ros::NodeHandle ph_{"~"};
    std::string solver_name_{"doogie_algorithms/RightHandSolverPlugin"};
    pluginlib::ClassLoader<doogie_algorithms::BaseSolver> solver_loader_;
    boost::shared_ptr<doogie_algorithms::BaseSolver> solver_;
};

}  //namespace doogie_algorithms

#endif  // DOOGIE_ALGORITHMS_SOLVER_PLUGIN_LOADER_HPP_