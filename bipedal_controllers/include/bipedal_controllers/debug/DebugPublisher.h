/**
 * @file DebugPublisher.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <ros/ros.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_bipedal_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>


namespace ocs2 {
namespace bipedal_robot {

class DebugPublisher {
public:
  DebugPublisher(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle);
  ~DebugPublisher() = default;

  void update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured);

private:

  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  PinocchioInterface pinocchioInterfaceMeasured_;
  PinocchioInterface pinocchioInterfaceDesired_;
  const CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;
  CentroidalModelPinocchioMapping mapping_;

  // ros publishers
  ros::Publisher basePoseDesiredPublisher_;
  ros::Publisher basePoseMeasuredPublisher_;
  std::vector<ros::Publisher> feetPoseDesiredPublishers_;
  std::vector<ros::Publisher> feetPoseMeasuredPublishers_;
  ros::Publisher jointDesiredPublisher_;
  ros::Publisher jointMeasuredPublisher_;

  // generalized coordinates
  // q = [q_b^T, q_j^T]^T
  // q_b is the floating base position and orientation
  // q_j is the joint position
  vector_t qMeasured_, qDesired_;

  // generalized velocities
  // v = [global_base_velocity_linear, global_base_velocity_angular, joint_velocities]^T
  vector_t vMeasured_, vDesired_;


};


} // namespace bipedal_robot
}  // namespace ocs2