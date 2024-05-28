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
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;

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

  /**
   * @brief Set the Swing Leg P D Gains object
   *  used in dynamic reconfigure
   * @param kp P gain for swing leg tracking task
   * @param kd D gain for swing leg tracking task
   */
  void setSwingLegPDGains(scalar_t kp, scalar_t kd) {
    swingKp_ = kp;
    swingKd_ = kd;
  }

  /**
   * @brief Set the Base P D Gains object
   *  used in dynamic reconfigure
   * @param baseKp P gain for base acceleration task
   * @param baseKd D gain for base acceleration task
   */
  void setBasePDGains(const Vector6& baseKp, const Vector6& baseKd) {
    baseKp_ = baseKp;
    baseKd_ = baseKd;
  }

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
                                scalar_t period);
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
  // v = [global_base_velocity_linear, global_base_velocity_angular, joint_velocities]^T
  vector_t vMeasured_;

  vector_t inputLast_;
  
  matrix_t j_;    // contact Jacobian
  matrix_t dj_;   // time derivative of contact Jacobian
  Matrix6x baseJ_, baseDj_; // base Jacobian and its time derivative

  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
  Vector6 baseKp_, baseKd_;
  


};



} // namespace bipedal_robot
}



