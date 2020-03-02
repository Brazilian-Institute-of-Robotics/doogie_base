#ifndef DOOGIE_CONTROL_PID_CONTROLLER_HPP_
#define DOOGIE_CONTROL_PID_CONTROLLER_HPP_

#include <string>

#include <control_toolbox/pid.h>

namespace doogie_control {
    
class PIDController{

 public:

  PIDController(const std::string& controller_name);
  ~PIDController();
  void initialize();
  void setTolerance(double tolerance);
  void setSetPoint(double setpoint);
  const double getSetPoint() const;
  const double getError() const;
  const double getAbsoluteError() const;
  bool isToleranceReached();
  void reset();
  double computeError(double actual_value);
  double computeAbsoluteError(double actual_value);
  double computeControlAction(double error);
  double computeControlAction(double setpoint, double actual_value);

 private:

  ros::NodeHandle pid_nh;

  control_toolbox::Pid pid_;
  
  double tolerance_{0};
  double setpoint_{0};
  double error_{0};
  double control_action_{0};

  ros::Time last_time_;
};

}  // namespace doogie_control

#endif  // DOOGIE_CONTROL_PID_CONTROLLER_HPP_
