#ifndef DOOGIE_LOCALIZATION_BASE_LOCALIZATION_HPP_
#define DOOGIE_LOCALIZATION_BASE_LOCALIZATION_HPP_

#include <string>

#include <ros/ros.h>

namespace doogie_localization {
    
class BaseLocalization {

 public:
  BaseLocalization();
  virtual ~BaseLocalization() = default;
  virtual double getCurrentXPosition() = 0;
  virtual double getCurrentYPosition()= 0;
  virtual double getCurrentYawOrientation() = 0;
  virtual double getCurrentNormalizedYawOrientation() = 0;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pn_{"~"};
};

using BaseLocalizationPtr = boost::shared_ptr<doogie_localization::BaseLocalization>;

}  // namespace doogie_localization

#endif  // DOOGIE_LOCALIZATION_BASE_LOCALIZATION_HPP_
