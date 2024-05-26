/**
 * @file InitialJointController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bipedal_controllers/InitialJointController.h"
#include "ocs2_bipedal_robot/common/ModelSettings.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

namespace ocs2{
namespace bipedal_robot{

bool InitialJointPositionController::init(HybridJointInterface* hw, ros::NodeHandle &nh){
  std::string taskFile, urdfFile, referenceFile;
  nh.getParam("/urdfFile", urdfFile);
  nh.getParam("/taskFile", taskFile);
  nh.getParam("/referenceFile", referenceFile);

  // check file exits
  boost::filesystem::path urdfPath(urdfFile);
  if(!boost::filesystem::exists(urdfPath)){
    throw std::invalid_argument("[InitialJointPositionController] The URDF file " + urdfFile + " does not exist!");
  }
  boost::filesystem::path referencePath(referenceFile);
  if(!boost::filesystem::exists(referencePath)){
    throw std::invalid_argument("[InitialJointPositionController] The reference file " + referenceFile + " does not exist!");
  }
  // check file exits
  boost::filesystem::path taskPath(taskFile);
  if(!boost::filesystem::exists(taskPath)){
    throw std::invalid_argument("[InitialJointPositionController] The task file " + taskFile + " does not exist!");
  }

  // load default joint states
  bool verbose = false;
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  jointNames_ = modelSettings_.jointNames;
  numJoints_ = jointNames_.size();
  auto defaultJointState = centroidal_model::loadDefaultJointState(numJoints_, referenceFile);
  initialDesiredJointPos_.resize(numJoints_);
  for (size_t i = 0; i < numJoints_; ++i){
    initialDesiredJointPos_[i] = defaultJointState[i];
  }

  // load urdf
  urdfTree_ = ::urdf::parseURDFFile(urdfFile);
  if (urdfTree_ == nullptr) {
    throw std::invalid_argument("[InitialJointPositionController] The file " + urdfFile + " does not contain a valid URDF model!");
  }
  // load upper and lower limits for each joint
  upperLimit_.resize(numJoints_);
  lowerLimit_.resize(numJoints_);
  for(size_t i = 0; i<numJoints_; i++){
    std::string name = jointNames_[i];
    auto jointUrdf = urdfTree_->getJoint(name);
    if(jointUrdf == nullptr){
      ROS_ERROR_STREAM("[InitialJointPositionController] Could not find joint " << name << " in URDF model.");
      return false;
    }
    if(jointUrdf->type != ::urdf::Joint::REVOLUTE){
      ROS_ERROR_STREAM("[InitialJointPositionController] Unsupported joint type for joint " << name);
      return false;
    }
    upperLimit_[i] = jointUrdf->limits->upper;
    lowerLimit_[i] = jointUrdf->limits->lower;
  }

  // // debug print 
  // std::cerr << "EffortJointInterface joint names: ";
  // auto debug_names = hw->getNames();  
  // for(auto name : debug_names){
  //   std::cerr << name << " ";
  // }
  // std::cerr << std::endl;

  // ros control
  jointHandles_.clear();
  pid_gains_.resize(numJoints_);
  for(size_t i = 0; i < numJoints_; i++){
    // load joints handle
    try{
      jointHandles_.push_back(hw->getHandle(jointNames_[i]));
    } catch (const hardware_interface::HardwareInterfaceException & e){
      ROS_ERROR_STREAM("[InitialJointPositionController] " << e.what());
      return false;
    }
    // load pid controllers using gains set on parameter server
    auto pid_nh = ros::NodeHandle(nh, jointNames_[i] + "/pid");
    pid_nh.getParam("p", pid_gains_[i].p_gain_);
    pid_nh.getParam("i", pid_gains_[i].i_gain_);
    pid_nh.getParam("d", pid_gains_[i].d_gain_);
  }

  return true;
}

// ###############################################################################################################
// ###############################################################################################################
// ###############################################################################################################

void InitialJointPositionController::update(const ros::Time& time, const ros::Duration& period){

  // std::cerr << "InitialJointPositionController::update: " << std::endl;

  for(size_t i = 0; i < numJoints_; i++){

    scalar_t desired_pos = initialDesiredJointPos_[i];

    // make sure joint is within limits
    enforceJointLimits(desired_pos, i);
    
    scalar_t kp = pid_gains_[i].p_gain_;
    scalar_t kd = pid_gains_[i].d_gain_;

    jointHandles_[i].setCommand(desired_pos, 0.0, kp, kd, 0.0);

    // std::cerr << "Joint " << jointNames_[i] << " desired_pos: " << desired_pos << " current_pos: " << current_pos << " commanded_effort: " << commanded_effort << std::endl;
  }
}

// ###############################################################################################################
// ###############################################################################################################
// ###############################################################################################################

void InitialJointPositionController::enforceJointLimits(double &command, size_t index){
  if(command > upperLimit_[index]){
    command = upperLimit_[index];
  } else if(command < lowerLimit_[index]){
    command = lowerLimit_[index];
  }
}



} // namespace bipedal_robot
} // namespace ocs2

PLUGINLIB_EXPORT_CLASS(ocs2::bipedal_robot::InitialJointPositionController, controller_interface::ControllerBase)