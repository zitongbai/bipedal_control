/**
 * @file FromTopiceEstimate.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief cheater estimation from topic
 * @version 0.1
 * @date 2024-05-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bipedal_estimation/FromTopicEstimate.h"

namespace ocs2{
namespace bipedal_robot{

FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) {
  ros::NodeHandle nh;
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  // debug print:
  std::cerr << "====================================" << std::endl;
  std::cerr << "current time" << ros::Time::now().toSec() << std::endl;
  std::cerr << "msg time " << odom.header.stamp.toSec() << std::endl;
  std::cerr << "z " << odom.pose.pose.position.z << std::endl;
  std::cerr << "====================================" << std::endl;

  updateAngular(quatToZyx(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
                Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
  updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  publishMsgs(odom);

  return rbdState_;
}

}  // namespace bipedal_robot
}  // namespace ocs2