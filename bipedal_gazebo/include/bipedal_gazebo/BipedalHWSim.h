/**
 * @file BipedalHWSim.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <deque>
#include <unordered_map>

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include "bipedal_common/hardware_interface/ContactSensorInterface.h"
#include "bipedal_common/hardware_interface/HybridJointInterface.h"

namespace ocs2 {
namespace bipedal_robot {

struct HybridJointData {
  hardware_interface::JointHandle joint_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct HybridJointCommand {
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct ImuData {
  gazebo::physics::LinkPtr linkPtr_;
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

/**
 * @brief a RobotHW inheriting from DefaultRobotHWSim
 *  Add some modifications to the DefaultRobotHWSim
 */
class LeggedHWSim : public gazebo_ros_control::DefaultRobotHWSim {
 public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

 private:
  void parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel);
  void parseContacts(XmlRpc::XmlRpcValue& contactNames);

  HybridJointInterface hybridJointInterface_;
  ContactSensorInterface contactSensorInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;

  gazebo::physics::ContactManager* contactManager_{};

  std::list<HybridJointData> hybridJointDatas_;
  std::list<ImuData> imuDatas_;
  std::unordered_map<std::string, std::deque<HybridJointCommand> > cmdBuffer_;
  std::unordered_map<std::string, bool> name2contact_;

  double delay_{};
};


}  // namespace bipedal_robot
}  // namespace ocs2

