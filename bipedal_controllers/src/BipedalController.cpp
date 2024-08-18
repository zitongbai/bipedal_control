/**
 * @file BipedalController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-25
 * @ref https://github.com/qiayuanl/legged_control
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "bipedal_controllers/BipedalController.h"
#include <ocs2_bipedal_robot_ros/gait/GaitReceiver.h>
#include <bipedal_estimation/FromTopicEstimate.h>
#include <bipedal_wbc/WeightedWbc.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/JointState.h>

namespace ocs2 {
namespace bipedal_robot {

BipedalController::~BipedalController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

bool BipedalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh){
  // initialize ocs2
  std::string taskFile, urdfFile, referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "bipedal_robot_interface.verbose", verbose);

  setupBipedalInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

  // visualization
  ::ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(bipedalInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      bipedalInterface_->getPinocchioInterface(), pinocchioMapping, bipedalInterface_->modelSettings().contactNames3DoF);
  // TODO: define a new class for visualization
  robotVisualizer_ = std::make_shared<BipedalRobotVisualizer>(
      bipedalInterface_->getPinocchioInterface(), bipedalInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  // TODO: self collision visualization

  // ros control interface
  // get joint handles
  auto * hybridJointInterface = robot_hw->get<HybridJointInterface>();
  auto legJointNames = bipedalInterface_->modelSettings().jointNames;
  for (const auto& jointName : legJointNames){
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName));
  }
  // get contact sensor handles
  // TODO: deal with contact sensor
  // auto * contactSensorInterface = robot_hw->get<ContactSensorInterface>();
  // for (const auto& contactName : bipedalInterface_->modelSettings().contactNames3DoF){
  //   contactHandles_.push_back(contactSensorInterface->getHandle(contactName));
  // }
  // get imu sensor handle
  auto * imuSensorInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  imuSensorHandle_ = imuSensorInterface->getHandle("base_imu");

  // state estimation
  setupStateEstimate(taskFile, verbose);

  // whole body control
  wbc_ = std::make_shared<WeightedWbc>(bipedalInterface_->getPinocchioInterface(), 
                                      bipedalInterface_->getCentroidalModelInfo(), 
                                      *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  safetyChecker_ = std::make_shared<SafetyChecker>(bipedalInterface_->getCentroidalModelInfo());

  // lowlevel controller
  defaultJointState_ = centroidal_model::loadDefaultJointState(legJointNames.size(), referenceFile);
  jointKp_.resize(legJointNames.size(), 0.0);
  jointKd_.resize(legJointNames.size(), 0.0);
  debugControlCmdPublisher_ = nh.advertise<sensor_msgs::JointState>(robotName_+"_joint_cmd_debug", 1);

  // dynamic reconfigure
  dynamicReconfigServer_.reset(new DynamicReconfigServer(ros::NodeHandle(controller_nh, "params")));
  dynamicReconfigCallback_ = boost::bind(&BipedalController::dynamicReconfigCallback, this, _1, _2);
  dynamicReconfigServer_->setCallback(dynamicReconfigCallback_);

  // debug 
  debugPublisher_ = std::make_shared<DebugPublisher>(bipedalInterface_->getPinocchioInterface(), 
                                                      bipedalInterface_->getCentroidalModelInfo(), 
                                                      *eeKinematicsPtr_, nh);

  return true;
}

void BipedalController::starting(const ros::Time& time){

  // initial state 
  currentObservation_.state.setZero(bipedalInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(bipedalInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  // debug: print initial observation state
  std::cerr << "===============================================================" << std::endl;
  std::cerr << "initial observation" << std::endl;
  std::cerr << "time: " << currentObservation_.time << std::endl;
  std::cerr << "base xyz " << currentObservation_.state.segment(6, 3).transpose() << std::endl;
  std::cerr << "base euler zyx " << currentObservation_.state.segment(9, 3).transpose() << std::endl;
  std::cerr << "left leg joints " << currentObservation_.state.segment(12, 5).transpose() << std::endl;
  std::cerr << "right leg joints " << currentObservation_.state.segment(17, 5).transpose() << std::endl;
  std::cerr << "===============================================================" << std::endl;
  

  // initial target trajectories
  auto init_target_traj_state = bipedalInterface_->getInitialState();

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  mpcMrtInterface_->reset();
  mpcMrtInterface_->resetMpcNode(target_trajectories);

  // Set the first observation and command and wait for optimization to finish
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok() && ros::master::check()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(bipedalInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  // debug: print initial optimized state and input
  mpcMrtInterface_->updatePolicy();
  vector_t initialOptimizedState, initialOptimizedInput;
  size_t plannedMode = 0;
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, initialOptimizedState, initialOptimizedInput, plannedMode);
  std::cerr << "===============================================================" << std::endl;
  std::cerr << "initialOptimizedState: " << std::endl;
  std::cerr << "base xyz " << initialOptimizedState.segment(6, 3).transpose() << std::endl;
  std::cerr << "base euler zyx " << initialOptimizedState.segment(9, 3).transpose() << std::endl;
  std::cerr << "left leg joints " << initialOptimizedState.segment(12, 5).transpose() << std::endl;
  std::cerr << "right leg joints " << initialOptimizedState.segment(17, 5).transpose() << std::endl;
  std::cerr << "initialOptimizedInput: " << std::endl;
  std::cerr << "contact forces " << initialOptimizedInput.segment(0, 12).transpose() << std::endl;
  std::cerr << "joint velocity " << initialOptimizedInput.segment(12, 10).transpose() << std::endl;
  std::cerr << "===============================================================" << std::endl;

  mpcRunning_ = true;

  // wbc
  wbc_->clearLastQpSol();
}

// ###############################################################################################################
// ###############################################################################################################
// ###############################################################################################################

void BipedalController::update(const ros::Time& time, const ros::Duration& period) {
  // State estimation
  updateStateEstimation(time, period);

  // update the current state for mpc mrt interface
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // load the latest MPC policy
  // the computation of solving MPC is done in a separate thread
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy and load them to `optimizedState` and `optimizedInput`
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  ROS_INFO_THROTTLE(1.0, "Mode: %zu", plannedMode);

  currentObservation_.input = optimizedInput;
  // visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

  // // debug: print optimizedState, optimizedInput
  // std::cerr << "===============================================================" << std::endl;
  // std::cerr << "time" << time.toSec() << std::endl;
  // std::cerr << "period" << period.toSec() << std::endl;
  // std::cerr << "optimizedState: " << optimizedState.transpose() << std::endl;
  // std::cerr << "optimizedInput: " << optimizedInput.transpose() << std::endl;
  // std::cerr << "measuredRbdState" << measuredRbdState_.transpose() << std::endl;
  // std::cerr << "plannedMode: " << plannedMode << std::endl;
  // std::cerr << "period: " << period.toSec() << std::endl;
  // std::cerr << "===============================================================" << std::endl;

  // if(plannedMode == ModeNumber::STANCE){
  //   optimizedState.setZero();
  //   optimizedInput.setZero();
  //   optimizedState.segment(6, 6) = currentObservation_.state.segment<6>(6);
  //   optimizedState.segment(6 + 6, bipedalInterface_->getCentroidalModelInfo().actuatedDofNum) = defaultJointState_;
  //   plannedMode = ModeNumber::STANCE;
  // }

  // Whole body control
  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  wbcTimer_.endTimer();

  // safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Bipedal Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  // low level control
  auto jointNum = bipedalInterface_->getCentroidalModelInfo().actuatedDofNum;
  vector_t torque = x.tail(jointNum);
  // vector_t wbcJointAcc = x.segment(6, jointNum);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, bipedalInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, bipedalInterface_->getCentroidalModelInfo());

  // scalar_t dt = period.toSec();
  // posDes = posDes + 0.5* wbcJointAcc * dt * dt;
  // velDes = velDes + wbcJointAcc * dt;

  for (size_t j = 0; j < jointNum; ++j) {
    scalar_t kp = jointKp_[j];
    scalar_t kd = jointKd_[j];
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), kp, kd, torque(j));
  }

  // for debug
  sensor_msgs::JointState jointStateCmdMsg;
  jointStateCmdMsg.name = bipedalInterface_->modelSettings().jointNames;
  jointStateCmdMsg.position.resize(jointNum);
  jointStateCmdMsg.velocity.resize(jointNum);
  jointStateCmdMsg.effort.resize(jointNum);
  for(size_t i=0; i < jointNum; ++i){
    jointStateCmdMsg.position[i] = posDes(i);
    jointStateCmdMsg.velocity[i] = velDes(i);
    jointStateCmdMsg.effort[i] = torque(i);
  }
  jointStateCmdMsg.header.stamp = time;
  debugControlCmdPublisher_.publish(jointStateCmdMsg);

  debugPublisher_->update(optimizedState, optimizedInput, measuredRbdState_);

  // TODO: self collision visualization

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

// ###############################################################################################################
// ###############################################################################################################
// ###############################################################################################################

void BipedalController::setupBipedalInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose){
  bipedalInterface_ = std::make_shared<BipedalRobotInterface>(taskFile, urdfFile, referenceFile);
}

// ###############################################################################################################
// ###############################################################################################################
// ###############################################################################################################


void BipedalController::setupMpc(){
  ::ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr = std::make_shared<GaitReceiver>(
      nh, bipedalInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName_);

  // ROS ReferenceManager
  // it has targetTrajectoriesSubscriber_ subscribing to "topicPrefix_mpc_target"
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName_, bipedalInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);

  // MPC
  mpc_ = std::make_shared<SqpMpc>(bipedalInterface_->mpcSettings(), 
                                  bipedalInterface_->sqpSettings(), 
                                  bipedalInterface_->getOptimalControlProblem(), 
                                  bipedalInterface_->getInitializer());
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

  // rigid body conversions
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(
              bipedalInterface_->getPinocchioInterface(), 
              bipedalInterface_->getCentroidalModelInfo());

  // observation publisher
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1);
}

void BipedalController::setupMrt(){
  // MPC_MRT_Interface
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&bipedalInterface_->getRollout());

  mpcTimer_.reset();

  controllerRunning_ = true;
  /**
   * @brief Launch the computation of the MPC in a separate thread.
   *  This thread will be triggered at a given frequency and execute an 
   *  optimization based on the latest available observation.
   */
  mpcThread_ = std::thread([&](){
    while (controllerRunning_){
      try {
        executeAndSleep(
          [&](){
            if (mpcRunning_){
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
            }

          }, bipedalInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e){
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(bipedalInterface_->sqpSettings().threadPriority, mpcThread_);
}

void BipedalController::setupStateEstimate(const std::string& taskFile, bool verbose){
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(bipedalInterface_->getPinocchioInterface(), 
                                                            bipedalInterface_->getCentroidalModelInfo(), 
                                                            *eeKinematicsPtr_);
}

void BipedalController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contactFlag;
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for(size_t i = 0; i < hybridJointHandles_.size(); i++){
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }

  // TODO: contact interface
  // for(size_t i = 0; i < contactHandles_.size(); i++){
  //   contactFlag[i] = contactHandles_[i].isContact();
  // }
  // TODO: update contactFlag
  contactFlag = modeNumber2StanceLeg(ModeNumber::STANCE);

  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  // prepare data for state estimation
  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  // update state estimate
  measuredRbdState_ = stateEstimate_->update(time, period);

  // convert to observation form
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.time += period.toSec();
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

void BipedalController::dynamicReconfigCallback(bipedal_controllers::BipedalControllerParamsConfig& config, uint32_t level){
  // update wbc parameters
  // PD gains for base accel task
  Eigen::Matrix<scalar_t, 6, 1> baseKp, baseKd;
  baseKp << config.base_position_x_kp, config.base_position_y_kp, config.base_position_z_kp,
            config.base_orientation_x_kp, config.base_orientation_y_kp, config.base_orientation_z_kp;
  baseKd << config.base_position_x_kd, config.base_position_y_kd, config.base_position_z_kd,
            config.base_orientation_x_kd, config.base_orientation_y_kd, config.base_orientation_z_kd;
  wbc_->setBasePDGains(baseKp, baseKd);
  // PD gains for swing leg task
  wbc_->setSwingLegPDGains(config.swing_kp, config.swing_kd);
  // task weights
  wbc_->setWeights(config.swing_leg_weight, config.base_accel_weight, config.contact_force_weight);

  // update joint controller pd gains
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
}

} // namespace bipedal_robot
}  // namespace ocs2

PLUGINLIB_EXPORT_CLASS(ocs2::bipedal_robot::BipedalController, controller_interface::ControllerBase)
