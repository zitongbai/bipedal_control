/**
 * @file WbcBase.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-22
 * @ref https://github.com/qiayuanl/legged_control
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "bipedal_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2{
namespace bipedal_robot{

WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      rbdConversions_(pinocchioInterface, info_),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) {
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                         scalar_t /*period*/) {
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired);

  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) {
  /**
   * @brief rbdStateMeasured: [euler_zyx, pos, joint_pos, angular_vel, linear_vel, joint_vel]
   *                           3          + 3  + n_j      + 3          + 3         + n_j
   */
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);    // floating base pos
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();    // floating base orientation, euler angle zyx
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);   // joint pos
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);  // floating base linear vel
	/**
	 * @brief get time derivative of euler angles zyx
	 * 		note that it is not the same as the angular velocity
	 * 		you can refer to *ETH Robot Dynamics Notes* 2.5.1
	 */
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), // floating base orientation, euler angle zyx
      rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum)  // floating base angular vel
    ); 
	// joint velocity
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
	/**
	 * @brief Computes the upper triangular part of the joint space inertia matrix M by
	 *	using the Composite Rigid Body Algorithm (Chapter 6, Rigid-Body Dynamics Algorithms, R. Featherstone, 2008).
	 *	The result is accessible through data.M.
	 */
  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
	/**
	 * @brief Computes the nonlinear effects (gravity, Coriolis and centrifugal effects) in the joints space.
	 * 		that is, b(q, u) + g(q) in Lagrangian dynamics.
	 * 		The result is accessible through data.nle.
	 */
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

	/**
	 * @brief Computes the contact Jacobian in pinocchio::LOCAL_WORLD_ALIGNED
	 */
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }
	/**
	 * @brief Computes the time variation of the contact Jacobian
	 * 	For not contact motion task
	 */
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  /**
   * @brief Computes the base Jacobian and its time derivative
   *  in pinocchio::LOCAL_WORLD_ALIGNED
   *  For base accel task (base motion tracking task)
   */
  baseJ_.setZero(6, info_.generalizedCoordinatesNum);
  baseDj_.setZero(6, info_.generalizedCoordinatesNum);
  // get base link name
  std::string baseLinkName = model.frames[2].name; // 0: universe, 1: root_joint
  pinocchio::getFrameJacobian(model, data, model.getFrameId(baseLinkName), pinocchio::LOCAL_WORLD_ALIGNED, baseJ_);
  pinocchio::getFrameJacobianTimeVariation(model, data, model.getFrameId(baseLinkName), pinocchio::LOCAL_WORLD_ALIGNED, baseDj_);
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired) {
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  // the first 6 elements of stateDesired are the collection of the normalized centroidal momentum
  // the next info_.generalizedCoordinatesNum elements are the generalized coordinates
  // helper function getPinocchioJointPosition is to extract the generalized coordinates from the state
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  pinocchio::forwardKinematics(model, data, qDesired);
  pinocchio::computeJointJacobians(model, data, qDesired);
  pinocchio::updateFramePlacements(model, data);
  // update the centroidal momentum matrix (CMM)
  // ref: Centroidal dynamics of a humanoid robot
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
  // velocity of floating base can be computed by the first 6 elements of the state
  // velocity of the joints can be extracted from the input
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired, vDesired);
}

/********************************************************************************************/
/* Equality Constraints */
/* For more details, refer to README.md */
/********************************************************************************************/

Task WbcBase::formulateFloatingBaseEomTask() {
  auto& data = pinocchioInterfaceMeasured_.getData();

  // selection matrix
  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
  vector_t b = -data.nle;

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateContactNoMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}


Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

  const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info_.actuatedDofNum);
  const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
  Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  centroidalMomentumRate.noalias() -= ADot * vDesired;
  centroidalMomentumRate.noalias() -= Aj * jointAccel;

  Vector6 b = AbInv * centroidalMomentumRate;

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateBaseAccelPDTask(const vector_t& stateDesired, 
                                const vector_t& inputDesired, 
                                scalar_t period){
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  // // set base Jacobian in a
  // a.block(3, 0, 3, info_.generalizedCoordinatesNum) = baseJ_.block(3, 0, 3, info_.generalizedCoordinatesNum);

  // joint acceleration
  vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;

  // desired base pose, velocity, and acceleration in world frame
  // basePose: [base position, base orientation (EulerAngles-ZYX)] expressed in the world frame
  // baseVelocity: [base linear velocity, base angular velocity] expressed in the world frame
  // baseAcceleration: [base linear acceleration, base angular acceleration] expressed in the world frame
  Vector6 desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration;
  rbdConversions_.computeBaseKinematicsFromCentroidalModel(
    stateDesired, inputDesired, jointAccel,   // in 
    desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration // out
  );
  
  // measured base pose, velocity in world frame
  Vector6 measuredBasePose, measuredBaseVelocity;
  measuredBasePose = qMeasured_.segment<6>(0);
  measuredBaseVelocity = vMeasured_.segment<6>(0);
  // convert to angular velocity from time derivative of euler angles zyx
  measuredBaseVelocity.segment<3>(3) = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
    measuredBasePose.segment<3>(3), measuredBaseVelocity.segment<3>(3)
  );

  vector3_t basePositionError = desiredBasePose.head<3>() - measuredBasePose.head<3>();
  vector3_t baselLinearVelocityError = desiredBaseVelocity.head<3>() - measuredBaseVelocity.head<3>();
  
  vector3_t baseOrientationError = rotationErrorInWorld<scalar_t>(
    getRotationMatrixFromZyxEulerAngles<scalar_t>(desiredBasePose.segment<3>(3)),
    getRotationMatrixFromZyxEulerAngles<scalar_t>(measuredBasePose.segment<3>(3))
  );
  vector3_t baseAngularVelocityError = desiredBaseVelocity.head<3>(3) - measuredBaseVelocity.head<3>(3);

  Vector6 b;
  b.setZero();
  b.head<3>() = desiredBaseAcceleration.head<3>() 
          + baseKp_.head<3>().cwiseProduct(basePositionError) 
          + baseKd_.head<3>().cwiseProduct(baselLinearVelocityError);
  // b.tail<3>() = desiredBaseAcceleration.tail<3>() 
  //         + baseKp_.tail<3>().cwiseProduct(baseOrientationError) 
  //         + baseKd_.tail<3>().cwiseProduct(baseAngularVelocityError)
  //         - baseDj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateStanceBaseAccelTask(){
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  Vector6 b;
  b.setZero();

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateSwingLegTask() {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}


/********************************************************************************************/
/* Equality Tasks and Inequality Tasks */
/* For more details, refer to README.md */
/********************************************************************************************/

/**
 * @todo add heading, lateral, and normal directions as input
 *  modulate the normal component of contact forces
 * @ref Perception-less Terrain Adaptation through Whole Body Control and Hierarchical Optimization
 */
Task WbcBase::formulateFrictionConeTask() {
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();
  
  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_;  // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}

/********************************************************************************************/
/* Inequality Tasks */
/* For more details, refer to README.md */
/********************************************************************************************/

Task WbcBase::formulateTorqueLimitsTask() {
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  int numJointsPerLeg = info_.actuatedDofNum / 2;
  for (size_t l = 0; l < 2 * 2; ++l) {
    // f.segment<3>(3 * l) = torqueLimits_;
    f.segment(numJointsPerLeg * l, numJointsPerLeg) = torqueLimits_;
  }

  return {matrix_t(), vector_t(), d, f};
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 2); // 2 means two legs
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### hip yaw, hip roll , hip pitch, knee, ankle: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);

  loadData::loadEigenMatrix(taskFile, "baseAccelPDTask.baseKp", baseKp_);
  loadData::loadEigenMatrix(taskFile, "baseAccelPDTask.baseKd", baseKd_);
}



}   // namespace bipedal_robot
}   // namespace ocs2
