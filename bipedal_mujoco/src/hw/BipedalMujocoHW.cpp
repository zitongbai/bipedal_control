/**
 * @file BipedalMujocoHW.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bipedal_mujoco/hw/BipedalMujocoHW.h"

namespace ocs2 {
namespace bipedal_robot {

BipedalMujocoHW::BipedalMujocoHW(mjModel *model, mjData *data, ::mujoco::Simulate & sim) 
    : mj_model_(model), mj_data_(data), sim_(sim) {
  jointNum_ = mj_model_->nu;
  jointSensorNum_ = jointSensorCoeff_ * jointNum_;

  // resize the joint data vector
  jointData_.resize(jointNum_);
}

bool BipedalMujocoHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    if(!BipedalHW::init(root_nh, robot_hw_nh)){
        return false;
    }

    // // todo: power limit
    // robot_hw_nh.getParam("power_limit", powerLimit_);

    bool initSuccess = true;
    initSuccess &= setupJoints();
    initSuccess &= setupImu();

    return initSuccess;
}

bool BipedalMujocoHW::setupJoints(){
  // check if the model is still valid
  if (mj_model_ == nullptr || mj_data_ == nullptr) {
    ROS_ERROR("[BipedalMujocoHW] Model or data is not valid");
    return false;
  }

  for(size_t i=0; i<jointNum_; i++){
    // get joint name and check it
    const char * jntName = mj_id2name(mj_model_, mjtObj::mjOBJ_ACTUATOR, i);
    if(jntName == nullptr){
      ROS_ERROR("[BipedalMujocoHW] Joint name is not valid");
      return false;
    }
    // register joint state handle to joint state interface
    hardware_interface::JointStateHandle jntStateHandle(jntName, &jointData_[i].pos_, &jointData_[i].vel_, &jointData_[i].tau_);
    jointStateInterface_.registerHandle(jntStateHandle);
    // register joint handle to hybrid joint interface
    HybridJointHandle jntHandle(jntStateHandle, &jointData_[i].posDes_, &jointData_[i].velDes_, &jointData_[i].kp_, &jointData_[i].kd_, &jointData_[i].ff_);
    hybridJointInterface_.registerHandle(jntHandle);
  }
  return true;
}

bool BipedalMujocoHW::setupImu(){
  // check if the model is still valid
  if (mj_model_ == nullptr || mj_data_ == nullptr) {
    ROS_ERROR("[BipedalMujocoHW] Model or data is not valid");
    return false;
  }

  hardware_interface::ImuSensorHandle imuSensorHandle("base_imu", "base_imu", 
                                                      imuData_.ori_, imuData_.oriCov_, 
                                                      imuData_.angularVel_, imuData_.angularVelCov_, 
                                                      imuData_.linearAcc_, imuData_.linearAccCov_);
  imuSensorInterface_.registerHandle(imuSensorHandle);
  
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

void BipedalMujocoHW::read(const ros::Time& time, const ros::Duration& period) {
  // check if the model is still valid
  if (mj_model_ == nullptr || mj_data_ == nullptr) {
    ROS_ERROR("Model or data is not valid");
    return;
  }
  {
    // lock the sim mutex
    const std::unique_lock<std::recursive_mutex> lock(sim_.mtx);

    // read joint sensor data
    for(size_t i = 0; i<jointNum_; i++){
      jointData_[i].pos_ = mj_data_->sensordata[i];
      jointData_[i].vel_ = mj_data_->sensordata[i + jointNum_];
      jointData_[i].tau_ = mj_data_->sensordata[i + 2 * jointNum_];
    }

    // read imu sensor data
    // quaternion convention for mujoco: w, x, y, z
    imuData_.ori_[0] = mj_data_->sensordata[jointSensorNum_ + 0];
    imuData_.ori_[1] = mj_data_->sensordata[jointSensorNum_ + 1];
    imuData_.ori_[2] = mj_data_->sensordata[jointSensorNum_ + 2];
    imuData_.ori_[3] = mj_data_->sensordata[jointSensorNum_ + 3];

    imuData_.angularVel_[0] = mj_data_->sensordata[jointSensorNum_ + 4];
    imuData_.angularVel_[1] = mj_data_->sensordata[jointSensorNum_ + 5];
    imuData_.angularVel_[2] = mj_data_->sensordata[jointSensorNum_ + 6];

    imuData_.linearAcc_[0] = mj_data_->sensordata[jointSensorNum_ + 7];
    imuData_.linearAcc_[1] = mj_data_->sensordata[jointSensorNum_ + 8];
    imuData_.linearAcc_[2] = mj_data_->sensordata[jointSensorNum_ + 9];

    // read contact data
    // todo: contact sensor

  } // end of lock

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  // notice that we called BipedalMujocoHW::read first and then BipedalMujocoHW::write
  for(size_t i=0; i<jointNum_; i++){
    jointData_[i].posDes_ = jointData_[i].pos_;
    jointData_[i].velDes_ = jointData_[i].vel_;
    jointData_[i].ff_ = 0.0;
    jointData_[i].kp_ = 0.0;
    jointData_[i].kd_ = 0.0;
  }

}

void BipedalMujocoHW::write(const ros::Time& time, const ros::Duration& period) {
  // check if the model is still valid
  if (mj_model_ == nullptr || mj_data_ == nullptr) {
    ROS_ERROR("Model or data is not valid");
    return;
  }
  // write data to mujoco
  for(size_t i=0; i<jointNum_; i++){
    mj_data_->ctrl[i] = jointData_[i].ff_ + 
                        jointData_[i].kp_ * (jointData_[i].posDes_ - mj_data_->sensordata[i]) +
                        jointData_[i].kd_ * (jointData_[i].velDes_ - mj_data_->sensordata[i + jointNum_]);
  }
}




}  // namespace bipedal_robot
}  // namespace ocs2

