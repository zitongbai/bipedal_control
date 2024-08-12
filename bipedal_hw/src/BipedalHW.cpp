/**
 * @file BipedalHW.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//
// Created by qiayuan on 1/24/22.
//


#include "bipedal_hw/BipedalHW.h"

namespace ocs2{
namespace bipedal_robot {
bool BipedalHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/) {
  // if (!loadUrdf(root_nh)) {
  //   ROS_ERROR("Error occurred while setting up urdf");
  //   return false;
  // }

  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  // TODO: deal with contact sensor
  // registerInterface(&contactSensorInterface_);

  return true;
}

bool BipedalHW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  rootNh.getParam("bipedal_robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

}  // namespace bipedal
}
