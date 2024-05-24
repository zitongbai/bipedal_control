/**
 * @file FromTopiceEstimate.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief cheater estimation from topic
 * @version 0.1
 * @date 2024-05-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "bipedal_estimation/StateEstimateBase.h"
#include <realtime_tools/realtime_buffer.h>

namespace ocs2{
namespace bipedal_robot{

class FromTopicStateEstimate : public StateEstimateBase {
public:
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics);

  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                 const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                 const matrix3_t& linearAccelCovariance) override{};

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace bipedal_robot
}  // namespace ocs2