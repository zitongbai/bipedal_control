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

  jointKd_.resize(numJoints_);
  jointKp_.resize(numJoints_);

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

  // ros control
  jointHandles_.clear();
  for(size_t i = 0; i < numJoints_; i++){
    // load joints handle
    try{
      jointHandles_.push_back(hw->getHandle(jointNames_[i]));
    } catch (const hardware_interface::HardwareInterfaceException & e){
      ROS_ERROR_STREAM("[InitialJointPositionController] " << e.what());
      return false;
    }
  }

  // dynamic reconfigure
  dynamicReconfigServer_.reset(new DynamicReconfigServer(ros::NodeHandle(nh, "params")));
  dynamicReconfigCallback_ = boost::bind(&InitialJointPositionController::dynamicReconfigCallback, this, _1, _2);
  dynamicReconfigServer_->setCallback(dynamicReconfigCallback_);

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

    scalar_t kp = jointKp_[i];
    scalar_t kd = jointKd_[i];

    jointHandles_[i].setCommand(desired_pos, 0.0, kp, kd, 0.0);

    // std::cerr << "Joint " << jointNames_[i] << " desired_pos: " << desired_pos ;
  }
  // std::cerr << std::endl;
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

// ###############################################################################################################
// ###############################################################################################################
// ###############################################################################################################
void InitialJointPositionController::dynamicReconfigCallback(bipedal_controllers::InitialJointControllerParamsConfig& config, uint32_t level){
  if(numJoints_ == 10){
    // each leg has 5 joints
    jointKp_[0] = config.leg_motor_1_kp;
    jointKp_[1] = config.leg_motor_2_kp;
    jointKp_[2] = config.leg_motor_3_kp;
    jointKp_[3] = config.leg_motor_4_kp;
    jointKp_[4] = config.leg_motor_5_kp;
    jointKp_[5] = config.leg_motor_1_kp;
    jointKp_[6] = config.leg_motor_2_kp;
    jointKp_[7] = config.leg_motor_3_kp;
    jointKp_[8] = config.leg_motor_4_kp;
    jointKp_[9] = config.leg_motor_5_kp;

    jointKd_[0] = config.leg_motor_1_kd;
    jointKd_[1] = config.leg_motor_2_kd;
    jointKd_[2] = config.leg_motor_3_kd;
    jointKd_[3] = config.leg_motor_4_kd;
    jointKd_[4] = config.leg_motor_5_kd;
    jointKd_[5] = config.leg_motor_1_kd;
    jointKd_[6] = config.leg_motor_2_kd;
    jointKd_[7] = config.leg_motor_3_kd;
    jointKd_[8] = config.leg_motor_4_kd;
    jointKd_[9] = config.leg_motor_5_kd;
  } else if (numJoints_ == 12){
    // each leg has 6 joints
    jointKp_[0] = config.leg_motor_1_kp;
    jointKp_[1] = config.leg_motor_2_kp;
    jointKp_[2] = config.leg_motor_3_kp;
    jointKp_[3] = config.leg_motor_4_kp;
    jointKp_[4] = config.leg_motor_5_kp;
    jointKp_[5] = config.leg_motor_6_kp;
    jointKp_[6] = config.leg_motor_1_kp;
    jointKp_[7] = config.leg_motor_2_kp;
    jointKp_[8] = config.leg_motor_3_kp;
    jointKp_[9] = config.leg_motor_4_kp;
    jointKp_[10] = config.leg_motor_5_kp;
    jointKp_[11] = config.leg_motor_6_kp;

    jointKd_[0] = config.leg_motor_1_kd;
    jointKd_[1] = config.leg_motor_2_kd;
    jointKd_[2] = config.leg_motor_3_kd;
    jointKd_[3] = config.leg_motor_4_kd;
    jointKd_[4] = config.leg_motor_5_kd;
    jointKd_[5] = config.leg_motor_6_kd;
    jointKd_[6] = config.leg_motor_1_kd;
    jointKd_[7] = config.leg_motor_2_kd;
    jointKd_[8] = config.leg_motor_3_kd;
    jointKd_[9] = config.leg_motor_4_kd;
    jointKd_[10] = config.leg_motor_5_kd;
    jointKd_[11] = config.leg_motor_6_kd;
  } else {
    ROS_ERROR_STREAM("[InitialJointPositionController] Dynamic reconfigure failed: joint number is not correct.");
  }
}


} // namespace bipedal_robot
} // namespace ocs2

PLUGINLIB_EXPORT_CLASS(ocs2::bipedal_robot::InitialJointPositionController, controller_interface::ControllerBase)