#include "doogie_control/pid_controller.hpp"

#include <cmath>

namespace doogie_control {

PIDController::PIDController(const ros::NodeHandle& pid_nh) : pid_nh_(pid_nh) {
    pid_.init(pid_nh_);
    last_time_ = ros::Time(0);
}

PIDController::~PIDController() {
    
}

void PIDController::setTolerance(double tolerance) {
    tolerance_ = tolerance;
}

void PIDController::setSetPoint(double setpoint) {
    setpoint_ = setpoint;
}

const double PIDController::getSetPoint() const {
    return setpoint_;
}

bool PIDController::isToleranceReached() {
    if (getAbsoluteError() <= tolerance_) {
        reset();
        return true;
    }
    return false;
}

void PIDController::reset() {
    last_time_ = ros::Time(0);
    setpoint_ = 0;
}

double PIDController::computeControlAction(double setpoint, double actual_value) {
    setpoint_ = setpoint;
    return computeControlAction(actual_value);
}

double PIDController::computeControlAction(double actual_value) {

    ros::Duration dt = ros::Time::now() - last_time_;
    last_time_ = ros::Time::now();
    return pid_.computeCommand(computeError(actual_value), dt);
}

double PIDController::computeError(double actual_value) {
    return error_ = setpoint_ - actual_value;
}

double PIDController::computeAbsoluteError(double actual_value) {
    error_ = computeError(actual_value);
    return std::abs(error_);
}

double const PIDController::getError() const {
    return error_;
}

double const PIDController::getAbsoluteError() const {
    return std::abs(error_);
}

}  // namespace doogie_control
