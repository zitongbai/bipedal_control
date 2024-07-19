/**
 * @file WeightedWbc.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-24
 * @ref https://github.com/qiayuanl/legged_control
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bipedal_wbc/WeightedWbc.h"

#include <qpOASES.hpp>

namespace ocs2 {
namespace bipedal_robot {

vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);
  plannedMode_ = mode;

  // Constraints
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // // debug print A, lbA, ubA
  // std::cerr << "A: " << std::endl;
  // std::cerr << A << std::endl;
  // std::cerr << "lbA: " << std::endl;
  // std::cerr << lbA << std::endl;
  // std::cerr << "ubA: " << std::endl;
  // std::cerr << ubA << std::endl;

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 20;

  // // debug print
  // std::cerr << "[WeightedWbc] try to solve QP" << std::endl;

  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());

  qpProblem.getPrimalSolution(qpSol.data());
  if(qpProblem.isSolved()){
    // // debug print
    // std::cerr << "qpSol: ";
    // std::cerr << "dot_v: " << qpSol.segment(0, 16).transpose() << std::endl;
    // std::cerr << "F: " << qpSol.segment(16, 12).transpose() << std::endl;
    // std::cerr << "tau: " << qpSol.segment(28, 10).transpose() << std::endl;
    // std::cerr << "[WeightedWbc] QP solved ~~~ ~~~ ~~~ ~~~" << std::endl;
    // save to lastQpSol_
    lastQpSol_ = qpSol;
  } else {
    std::cerr << "[WeightedWbc] QP not solved !!!" << std::endl;
    // qpSol.setZero(getNumDecisionVars());
    qpSol = lastQpSol_;
  }

  return qpSol;
}

Task WeightedWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() 
        + formulateTorqueLimitsTask() 
        + formulateFrictionConeTask() 
        // + formulateContactNoMotionTask()
        ;
}

Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  // if (plannedMode_ == ModeNumber::STANCE){
  //   return formulateStanceBaseAccelTask()* weightBaseAccel_;
  // } else {
    return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelPDTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
            formulateContactForceTask(inputDesired) * weightContactForce_;
  // }
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}


}  // namespace bipedal_robot
}  // namespace ocs2