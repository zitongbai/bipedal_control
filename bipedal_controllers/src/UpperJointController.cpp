/**
 * @file UpperJointController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-07-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bipedal_controllers/UpperJointController.h"
#include "ocs2_bipedal_robot/common/ModelSettings.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

namespace ocs2{
namespace bipedal_robot{

bool UpperJointController::init(HybridJointInterface* hw, ros::NodeHandle &nh){

  std::string taskFile, urdfFile, referenceFile;
  nh.getParam("/urdfFile", urdfFile);
  nh.getParam("/taskFile", taskFile);
  nh.getParam("/referenceFile", referenceFile);

  // check file exits
  boost::filesystem::path urdfPath(urdfFile);
  if(!boost::filesystem::exists(urdfPath)){
    throw std::invalid_argument("[UpperJointController] The URDF file " + urdfFile + " does not exist!");
  }
  boost::filesystem::path referencePath(referenceFile);
  if(!boost::filesystem::exists(referencePath)){
    throw std::invalid_argument("[UpperJointController] The reference file " + referenceFile + " does not exist!");
  }
  // check file exits
  boost::filesystem::path taskPath(taskFile);
  if(!boost::filesystem::exists(taskPath)){
    throw std::invalid_argument("[UpperJointController] The task file " + taskFile + " does not exist!");
  }

  // load urdf
  urdfTree_ = ::urdf::parseURDFFile(urdfFile);
  if (urdfTree_ == nullptr) {
    throw std::invalid_argument("[UpperJointController] The file " + urdfFile + " does not contain a valid URDF model!");
  }

  // load model settings
  bool verbose = false;
  // load upper joint names
  loadData::loadStdVector(taskFile, "model_settings.upperJointNames", jointNames_, verbose);
  numJoints_ = jointNames_.size();
  // TODO: make it more general
  desiredJointPos_ = std::vector<scalar_t>(numJoints_, 0.0);
  desiredJointPos_[4] = -1.2; // left_elbow_joint
  desiredJointPos_[8] = -1.2; // right_elbow_joint

  // load upper and lower limits for each joint
  upperLimit_.resize(numJoints_);
  lowerLimit_.resize(numJoints_);
  for(size_t i = 0; i<numJoints_; i++){
    std::string name = jointNames_[i];
    auto jointUrdf = urdfTree_->getJoint(name);
    if(jointUrdf == nullptr){
      ROS_ERROR_STREAM("[UpperJointController] Could not find joint " << name << " in URDF model.");
      return false;
    }
    if(jointUrdf->type != ::urdf::Joint::REVOLUTE){
      ROS_ERROR_STREAM("[UpperJointController] Unsupported joint type for joint " << name);
      return false;
    }
    upperLimit_[i] = jointUrdf->limits->upper;
    lowerLimit_[i] = jointUrdf->limits->lower;
  }

  // ros control 
  jointHandles_.clear();
  for(size_t i = 0; i < numJoints_; i++){
    // load joints handle
    try{
      jointHandles_.push_back(hw->getHandle(jointNames_[i]));
    } catch (const hardware_interface::HardwareInterfaceException & e){
      ROS_ERROR_STREAM("[UpperJointController] " << e.what());
      return false;
    }
  }

  return true;
}

void UpperJointController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/){

  for(size_t i = 0; i < numJoints_; i++){

    scalar_t desired_pos = desiredJointPos_[i];

    // make sure joint is within limits
    enforceJointLimits(desired_pos, i);

    scalar_t kp = 20.0;
    scalar_t kd = 3.0;

    jointHandles_[i].setCommand(desired_pos, 0.0, kp, kd, 0.0);

  }

}

void UpperJointController::enforceJointLimits(double &command, size_t index){
  if(command > upperLimit_[index]){
    command = upperLimit_[index];
  } else if(command < lowerLimit_[index]){
    command = lowerLimit_[index];
  }
}


} // namespace bipedal_robot
} // namespace ocs2

PLUGINLIB_EXPORT_CLASS(ocs2::bipedal_robot::UpperJointController, controller_interface::ControllerBase)
