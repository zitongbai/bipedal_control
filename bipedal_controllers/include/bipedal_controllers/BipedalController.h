/**
 * @file BipedalController.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-25
 * @ref https://github.com/qiayuanl/legged_control
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <bipedal_common/hardware_interface/ContactSensorInterface.h>
#include <bipedal_common/hardware_interface/HybridJointInterface.h>
#include <bipedal_estimation/StateEstimateBase.h>
#include <bipedal_wbc/WeightedWbc.h>
#include <ocs2_bipedal_robot/BipedalRobotInterface.h>
#include <ocs2_bipedal_robot_ros/visualization/BipedalRobotVisualizer.h>
#include <bipedal_controllers/SafetyChecker.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <bipedal_controllers/BipedalControllerParamsConfig.h>
#include <bipedal_controllers/debug/DebugPublisher.h>

#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <mutex>

namespace ocs2{
namespace bipedal_robot{

class BipedalController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface
                                                                              //  ,ContactSensorInterface
                                                                               > {
public:
  BipedalController() = default;
  ~BipedalController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { 
    mpcRunning_ = false; 
    wbc_->clearLastQpSol();
  }

  const std::string robotName_ = "bipedal_robot";

protected:
  /**
   * @brief update joint positions, velocties, etc. from ros control interface
   *  and prepare data for state estimation
   *  data flow: ros control --> state estimation --> currentObservation_
   * @param time 
   * @param period 
   */
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
  virtual void setupBipedalInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // Interface
  std::shared_ptr<BipedalRobotInterface> bipedalInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  
  // ros control interface
  std::vector<HybridJointHandle> hybridJointHandles_;
  // std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_; // length: 2 * generalized coordinate
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WeightedWbc> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // NMPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<BipedalRobotVisualizer> robotVisualizer_;
  ros::Publisher observationPublisher_;

  // low-level controller
  // std::vector<control_toolbox::Pid> pidControllers_;
  std::vector<scalar_t> jointKp_, jointKd_;
  vector_t defaultJointState_;
  ros::Publisher debugControlCmdPublisher_;

  // dynamic reconfigure
  typedef dynamic_reconfigure::Server<bipedal_controllers::BipedalControllerParamsConfig> DynamicReconfigServer;
  std::shared_ptr<DynamicReconfigServer> dynamicReconfigServer_;
  DynamicReconfigServer::CallbackType dynamicReconfigCallback_;
  void dynamicReconfigCallback(bipedal_controllers::BipedalControllerParamsConfig& config, uint32_t level);

  // debug 
  std::shared_ptr<DebugPublisher> debugPublisher_;

private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;

};

}  // namespace bipedal_robot
}  // namespace ocs2

