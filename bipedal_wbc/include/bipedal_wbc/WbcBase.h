/**
 * @file WbcBase.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief base class for whole-body control
 * @version 0.1
 * @date 2024-05-22
 * @ref https://github.com/qiayuanl/legged_control
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "bipedal_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_bipedal_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

namespace ocs2{
namespace bipedal_robot{

class WbcBase{
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

public:
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  /**
   * @brief given the desired state, input, and measured state, update
   * 
   * @param stateDesired [h^T, q^T]^T
   * @param inputDesired [F_c^T, v_j^T]^T
   * @param rbdStateMeasured [euler_zyx, pos, joint_pos, angular_vel, linear_vel, joint_vel]
   *                          3          + 3  + n_j      + 3          + 3         + n_j
   * @param mode 
   * @param period 
   * @return vector_t 
   */
  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }
  
  /**
   * @brief formulate the task for the equation of motion (EOM)
   *  Please refer to README.md for more details
   * @return Task 
   */
  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateContactNoMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateStanceBaseAccelTask();
  Task formulateBaseAccelPDTask(const vector_t& stateDesired, 
                                const vector_t& inputDesired, 
                                scalar_t period, 
                                const Vector6& pGains, 
                                const Vector6& dGains);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;

  /**
   * @brief number of decision variables
   *    x = [\dot u^T, F^T, \tau^T]^T
   *    where \dot u is the generalized acceleration, 
   *    F is the contact force, and \tau is the joint torque
   */
  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;
  CentroidalModelRbdConversions rbdConversions_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  // generalized coordinates
  // q = [q_b^T, q_j^T]^T
  // q_b is the floating base position and orientation
  // q_j is the joint position
  vector_t qMeasured_;

  // generalized velocities
  // u = [u_b^T, u_j^T]^T
  // u_b is the floating base linear and angular velocity
  // u_j is the joint velocity
  // note that u_b is not equal to the time derivative of q_b
  vector_t vMeasured_;

  vector_t inputLast_;
  
  matrix_t j_;    // contact Jacobian
  matrix_t dj_;   // time derivative of contact Jacobian

  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};

};



} // namespace bipedal_robot
}



