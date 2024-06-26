/**
 * @file DebugPublisher.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */


// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ocs2_bipedal_robot/gait/MotionPhaseDefinition.h"
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

// ROS
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>

#include <bipedal_controllers/debug/DebugPublisher.h>


namespace ocs2 {
namespace bipedal_robot {

DebugPublisher::DebugPublisher(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      centroidalModelInfo_(std::move(centroidalModelInfo)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()), 
      mapping_(centroidalModelInfo_){
  
  // ros publishers
  basePoseDesiredPublisher_ = nodeHandle.advertise<geometry_msgs::Pose>("debug_base_pose_desired", 1);
  basePoseMeasuredPublisher_ = nodeHandle.advertise<geometry_msgs::Pose>("debug_base_pose_measured", 1);
  feetPoseDesiredPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  feetPoseMeasuredPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  // the order of the feet names can be found in task.info: model_settings.contactNames3DoF
  const auto & feetNames = endEffectorKinematicsPtr_->getIds();
  for(size_t i=0; i<centroidalModelInfo_.numThreeDofContacts; i++){
    feetPoseDesiredPublishers_[i] = nodeHandle.advertise<geometry_msgs::Pose>("debug_foot_pose_desired/" + feetNames[i], 1);
    feetPoseMeasuredPublishers_[i] = nodeHandle.advertise<geometry_msgs::Pose>("debug_foot_pose_measured/" + feetNames[i], 1);
  }
  // the order of the joint names can be found in task.info: model_settings.jointNames
  jointDesiredPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("debug_joint_desired", 1);
  jointMeasuredPublisher_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("debug_joint_measured", 1);

  // reserve the size 
  qMeasured_ = vector_t(centroidalModelInfo_.generalizedCoordinatesNum);
  qDesired_ = vector_t(centroidalModelInfo_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(centroidalModelInfo_.generalizedCoordinatesNum);
  vDesired_ = vector_t(centroidalModelInfo_.generalizedCoordinatesNum);

}

void DebugPublisher::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured){

  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired);

  endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  // in the order of endEffectorKinematicsPtr_->getIds()
  std::vector<vector3_t> feetPosMeasured = endEffectorKinematicsPtr_->getPosition(vector_t());
  endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterfaceDesired_);
  // in the order of endEffectorKinematicsPtr_->getIds()
  std::vector<vector3_t> feetPosDesired = endEffectorKinematicsPtr_->getPosition(vector_t());

  geometry_msgs::Pose measuredFootMsg, desiredFootMsg;
  for(size_t i=0; i<centroidalModelInfo_.numThreeDofContacts; i++){
    // publish measured foot position
    measuredFootMsg.position.x = feetPosMeasured[i](0);
    measuredFootMsg.position.y = feetPosMeasured[i](1);
    measuredFootMsg.position.z = feetPosMeasured[i](2);
    feetPoseMeasuredPublishers_[i].publish(measuredFootMsg);
    
    // publish desired foot position
    desiredFootMsg.position.x = feetPosDesired[i](0);
    desiredFootMsg.position.y = feetPosDesired[i](1);
    desiredFootMsg.position.z = feetPosDesired[i](2);
    feetPoseDesiredPublishers_[i].publish(desiredFootMsg);
  }

  // publish measured base position
  geometry_msgs::Pose measuredBaseMsg;
  measuredBaseMsg.position.x = qMeasured_(0);
  measuredBaseMsg.position.y = qMeasured_(1);
  measuredBaseMsg.position.z = qMeasured_(2);
  measuredBaseMsg.orientation.z = qMeasured_(3);
  measuredBaseMsg.orientation.y = qMeasured_(4);
  measuredBaseMsg.orientation.x = qMeasured_(5);
  basePoseMeasuredPublisher_.publish(measuredBaseMsg);

  // publish desired base position
  geometry_msgs::Pose desiredBaseMsg;
  desiredBaseMsg.position.x = qDesired_(0);
  desiredBaseMsg.position.y = qDesired_(1);
  desiredBaseMsg.position.z = qDesired_(2);
  desiredBaseMsg.orientation.z = qDesired_(3);
  desiredBaseMsg.orientation.y = qDesired_(4);
  desiredBaseMsg.orientation.x = qDesired_(5);
  basePoseDesiredPublisher_.publish(desiredBaseMsg);

  // publish measured joint position
  std_msgs::Float64MultiArray measuredJointMsg;
  auto measuredJointPos = qMeasured_.tail(centroidalModelInfo_.actuatedDofNum);
  measuredJointMsg.data = std::vector<double>(measuredJointPos.data(), measuredJointPos.data() + measuredJointPos.size());
  jointMeasuredPublisher_.publish(measuredJointMsg);

  // publish desired joint position
  std_msgs::Float64MultiArray desiredJointMsg;
  auto desiredJointPos = qDesired_.tail(centroidalModelInfo_.actuatedDofNum);
  desiredJointMsg.data = std::vector<double>(desiredJointPos.data(), desiredJointPos.data() + desiredJointPos.size());
  jointDesiredPublisher_.publish(desiredJointMsg);

}

void DebugPublisher::updateMeasured(const vector_t& rbdStateMeasured) {

  /**
   * @brief rbdStateMeasured: [euler_zyx, pos, joint_pos, angular_vel, linear_vel, joint_vel]
   *                           3          + 3  + n_j      + 3          + 3         + n_j
   */
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);    // floating base pos
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();    // floating base orientation, euler angle zyx
  qMeasured_.tail(centroidalModelInfo_.actuatedDofNum) = rbdStateMeasured.segment(6, centroidalModelInfo_.actuatedDofNum);   // joint pos
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum + 3);  // floating base linear vel
	/**
	 * @brief get time derivative of euler angles zyx
	 * 		note that it is not the same as the angular velocity
	 * 		you can refer to *ETH Robot Dynamics Notes* 2.5.1
	 */
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), // floating base orientation, euler angle zyx
      rbdStateMeasured.segment<3>(centroidalModelInfo_.generalizedCoordinatesNum)  // floating base angular vel
    ); 
	// joint velocity
  vMeasured_.tail(centroidalModelInfo_.actuatedDofNum) = rbdStateMeasured.segment(centroidalModelInfo_.generalizedCoordinatesNum + 6, centroidalModelInfo_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // forward kinematics for measured
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);

}

void DebugPublisher::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired){
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  // helper function getPinocchioJointPosition is to extract the generalized coordinates from the state
  // not only leg joints, but also include base
  qDesired_ = mapping_.getPinocchioJointPosition(stateDesired);
  pinocchio::forwardKinematics(model, data, qDesired_);
  pinocchio::computeJointJacobians(model, data, qDesired_);
  pinocchio::updateFramePlacements(model, data);
  // update the centroidal momentum matrix (CMM)
  updateCentroidalDynamics(pinocchioInterfaceDesired_, centroidalModelInfo_, qDesired_);
  // velocity of floating base can be computed by the first 6 elements of the state
  // velocity of the joints can be extracted from the input
  vDesired_ = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);

}


} // namespace bipedal_robot
}  // namespace ocs2